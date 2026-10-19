#define _POSIX_C_SOURCE 200809L
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define MAX_BYTES 1000000u
#define MAX_SNAPSHOTS 8192u
#define FN(ret, name, ...) static ret (*name)(__VA_ARGS__)
FN(int, snd_pcm_open, void **, const char *, int, int);
FN(int, snd_pcm_close, void *);
FN(int, snd_pcm_set_params, void *, int, int, unsigned, unsigned, int, unsigned);
FN(long, snd_pcm_writei, void *, const void *, unsigned long);
FN(int, snd_pcm_state, void *);
FN(int, snd_pcm_start, void *);
FN(int, snd_pcm_nonblock, void *, int);
FN(int, snd_pcm_drain, void *);
FN(int, snd_pcm_drop, void *);
FN(int, snd_mixer_open, void **, int);
FN(int, snd_mixer_attach, void *, const char *);
FN(int, snd_mixer_selem_register, void *, void *, void *);
FN(int, snd_mixer_load, void *);
FN(int, snd_mixer_close, void *);
FN(void *, snd_mixer_first_elem, void *);
FN(void *, snd_mixer_elem_next, void *);
FN(const char *, snd_mixer_selem_get_name, void *);
FN(int, snd_mixer_selem_has_playback_volume, void *);
FN(int, snd_mixer_selem_has_capture_volume, void *);
FN(int, snd_mixer_selem_has_playback_switch, void *);
FN(int, snd_mixer_selem_has_capture_switch, void *);
FN(int, snd_mixer_selem_is_enumerated, void *);
FN(int, snd_mixer_selem_get_playback_volume, void *, int, long *);
FN(int, snd_mixer_selem_get_capture_volume, void *, int, long *);
FN(int, snd_mixer_selem_get_playback_switch, void *, int, int *);
FN(int, snd_mixer_selem_get_capture_switch, void *, int, int *);
FN(int, snd_mixer_selem_get_enum_item, void *, int, unsigned *);
FN(int, snd_mixer_selem_set_playback_volume, void *, int, long);
FN(int, snd_mixer_selem_set_capture_volume, void *, int, long);
FN(int, snd_mixer_selem_set_playback_switch, void *, int, int);
FN(int, snd_mixer_selem_set_capture_switch, void *, int, int);
FN(int, snd_mixer_selem_set_enum_item, void *, int, unsigned);
FN(int, snd_mixer_selem_get_playback_volume_range, void *, long *, long *);

static volatile sig_atomic_t stopping;
static void stop_requested(int sig) { (void)sig; stopping = 1; }
static double monotonic_s(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + t.tv_nsec / 1e9;
}
static void signals(void) {
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = stop_requested;
    sigemptyset(&action.sa_mask);
    sigaction(SIGTERM, &action, NULL); sigaction(SIGINT, &action, NULL); sigaction(SIGHUP, &action, NULL);
    signal(SIGPIPE, SIG_IGN);
}
static uint16_t le16(const unsigned char *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const unsigned char *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static int read_audio(unsigned char **buffer, unsigned char **pcm, size_t *frames) {
    unsigned char *data = malloc(MAX_BYTES + 1);
    if (!data) return -1;
    size_t count = 0; double deadline = monotonic_s() + 5;
    while (!stopping && monotonic_s() < deadline && count <= MAX_BYTES) {
        struct pollfd poller = { STDIN_FILENO, POLLIN, 0 };
        int ready = poll(&poller, 1, 100);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) break;
        if (!ready) continue;
        ssize_t n = read(STDIN_FILENO, data + count, MAX_BYTES + 1 - count);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0) break;
        if (!n) {
            if (count < 44 || count > MAX_BYTES || memcmp(data, "RIFF", 4) || memcmp(data + 8, "WAVE", 4) || (uint64_t)le32(data + 4) + 8 != count) break;
            int format_ok = 0; size_t pos = 12; unsigned char *samples = NULL; uint32_t sample_bytes = 0;
            while (pos + 8 <= count) {
                uint32_t bytes = le32(data + pos + 4);
                if (bytes > count - pos - 8) break;
                if (!memcmp(data + pos, "fmt ", 4)) {
                    if (bytes < 16) break;
                    const unsigned char *fmt = data + pos + 8;
                    format_ok = le16(fmt) == 1 && le16(fmt + 2) == 1 && le32(fmt + 4) == 16000 && le32(fmt + 8) == 32000 && le16(fmt + 12) == 2 && le16(fmt + 14) == 16;
                } else if (!memcmp(data + pos, "data", 4)) {
                    if (samples) break;
                    samples = data + pos + 8; sample_bytes = bytes;
                }
                pos += 8 + bytes + (bytes & 1u);
            }
            if (format_ok && samples && sample_bytes && sample_bytes % 2 == 0 && sample_bytes <= 16000u * 2 * 30 && pos == count) {
                *buffer = data; *pcm = samples; *frames = sample_bytes / 2; return 0;
            }
            break;
        }
        count += (size_t)n;
    }
    free(data); return -1;
}
static void *alsa_library;
static int load_alsa(void) {
    const char *name = "libasound.so.2";
#ifdef ROBOT_AUDIO_TESTING
    if (getenv("ROBOT_AUDIO_ALSA_LIBRARY")) name = getenv("ROBOT_AUDIO_ALSA_LIBRARY");
#endif
    void *lib = dlopen(name, RTLD_NOW | RTLD_LOCAL);
    if (!lib) return -1;
    alsa_library = lib;
#define LOAD(name) do { *(void **)(&name) = dlsym(lib, #name); if (!name) return -1; } while (0)
    LOAD(snd_pcm_open); LOAD(snd_pcm_close); LOAD(snd_pcm_set_params); LOAD(snd_pcm_writei);
    LOAD(snd_pcm_state); LOAD(snd_pcm_start); LOAD(snd_pcm_nonblock); LOAD(snd_pcm_drain); LOAD(snd_pcm_drop);
    LOAD(snd_mixer_open); LOAD(snd_mixer_attach); LOAD(snd_mixer_selem_register); LOAD(snd_mixer_load); LOAD(snd_mixer_close);
    LOAD(snd_mixer_first_elem); LOAD(snd_mixer_elem_next); LOAD(snd_mixer_selem_get_name);
    LOAD(snd_mixer_selem_has_playback_volume); LOAD(snd_mixer_selem_has_capture_volume);
    LOAD(snd_mixer_selem_has_playback_switch); LOAD(snd_mixer_selem_has_capture_switch); LOAD(snd_mixer_selem_is_enumerated);
    LOAD(snd_mixer_selem_get_playback_volume); LOAD(snd_mixer_selem_get_capture_volume);
    LOAD(snd_mixer_selem_get_playback_switch); LOAD(snd_mixer_selem_get_capture_switch); LOAD(snd_mixer_selem_get_enum_item);
    LOAD(snd_mixer_selem_set_playback_volume); LOAD(snd_mixer_selem_set_capture_volume);
    LOAD(snd_mixer_selem_set_playback_switch); LOAD(snd_mixer_selem_set_capture_switch); LOAD(snd_mixer_selem_set_enum_item);
    LOAD(snd_mixer_selem_get_playback_volume_range);
#undef LOAD
    return 0;
}
struct snapshot { void *element; int kind; int channel; long value; };
static int get_value(void *element, int kind, int channel, long *value) {
    int v; unsigned u;
    switch (kind) {
        case 0: return snd_mixer_selem_get_playback_volume(element, channel, value);
        case 1: return snd_mixer_selem_get_capture_volume(element, channel, value);
        case 2: if (snd_mixer_selem_get_playback_switch(element, channel, &v) < 0) return -1; *value = v; return 0;
        case 3: if (snd_mixer_selem_get_capture_switch(element, channel, &v) < 0) return -1; *value = v; return 0;
        case 4: if (snd_mixer_selem_get_enum_item(element, channel, &u) < 0) return -1; *value = u; return 0;
    }
    return -1;
}
static int set_value(const struct snapshot *s) {
    switch (s->kind) {
        case 0: return snd_mixer_selem_set_playback_volume(s->element, s->channel, s->value);
        case 1: return snd_mixer_selem_set_capture_volume(s->element, s->channel, s->value);
        case 2: return snd_mixer_selem_set_playback_switch(s->element, s->channel, (int)s->value);
        case 3: return snd_mixer_selem_set_capture_switch(s->element, s->channel, (int)s->value);
        case 4: return snd_mixer_selem_set_enum_item(s->element, s->channel, (unsigned)s->value);
    }
    return -1;
}
static int snapshot_mixer(void **mixer, struct snapshot *records, size_t *count) {
    if (snd_mixer_open(mixer, 0) < 0 || snd_mixer_attach(*mixer, "hw:0") < 0 || snd_mixer_selem_register(*mixer, NULL, NULL) < 0 || snd_mixer_load(*mixer) < 0) return -1;
    *count = 0;
    for (void *e = snd_mixer_first_elem(*mixer); e; e = snd_mixer_elem_next(e)) {
        int kinds[] = {snd_mixer_selem_has_playback_volume(e), snd_mixer_selem_has_capture_volume(e), snd_mixer_selem_has_playback_switch(e), snd_mixer_selem_has_capture_switch(e), snd_mixer_selem_is_enumerated(e)};
        for (int kind = 0; kind < 5; kind++) if (kinds[kind]) {
            for (int channel = 0; channel < 32; channel++) {
                long value;
                if (get_value(e, kind, channel, &value) < 0) continue;
                if (*count >= MAX_SNAPSHOTS) return -1;
                records[(*count)++] = (struct snapshot){e, kind, channel, value};
            }
        }
    }
    return 0;
}
static int set_output(void *mixer, double volume) {
    int found_volume = 0;
    for (void *e = snd_mixer_first_elem(mixer); e; e = snd_mixer_elem_next(e)) {
        const char *name = snd_mixer_selem_get_name(e);
        if (!strcmp(name, "Speaker Function") && snd_mixer_selem_is_enumerated(e)) {
            if (snd_mixer_selem_set_enum_item(e, 0, 1) < 0) return -1;
        }
        if (!strcmp(name, "Audio speaker out")) {
            if (snd_mixer_selem_set_playback_switch(e, 0, 1) < 0) return -1;
        }
        if (!strcmp(name, "speaker volume control")) {
            long lo, hi;
            if (snd_mixer_selem_get_playback_volume_range(e, &lo, &hi) < 0) return -1;
            if (snd_mixer_selem_set_playback_volume(e, 0, lo + (long)((hi - lo) * volume)) < 0) return -1;
            found_volume = 1;
        }
    }
    return found_volume ? 0 : -1;
}
static int play_worker(void *mixer, const unsigned char *pcm, size_t frames, double volume) {
    void *device = NULL;
    int err = snd_pcm_open(&device, "hw:0,0", 0, 0);
    if (err < 0) { puts(err == -EBUSY ? "BUSY" : "ERROR"); return err == -EBUSY ? 3 : 1; }
    if (snd_pcm_set_params(device, 2, 3, 1, 16000, 0, 100000) < 0 || set_output(mixer, volume) < 0) { snd_pcm_close(device); return 1; }
    size_t done = 0; int started = 0;
    while (done < frames && !stopping) {
        size_t n = frames - done; if (n > 512) n = 512;
        long written = snd_pcm_writei(device, pcm + done * 2, n);
        if (written < 0) break;
        if (!written) break;
        done += (size_t)written;
        if (!started) {
            if (snd_pcm_state(device) == 2 && snd_pcm_start(device) < 0) break;
            started = 1; puts("STARTED"); fflush(stdout);
        }
    }
    int result = 1;
    if (!stopping && done == frames && snd_pcm_nonblock(device, 0) >= 0 && snd_pcm_drain(device) >= 0) result = 0;
    snd_pcm_drop(device); snd_pcm_close(device);
    return stopping ? 130 : result;
}
static const char *lock_path(void) {
#ifdef ROBOT_AUDIO_TESTING
    if (getenv("ROBOT_AUDIO_LOCK")) return getenv("ROBOT_AUDIO_LOCK");
#endif
    return "/tmp/robot-audio.lock";
}
static int valid_token(const char *token) {
    if (strlen(token) != 32) return 0;
    for (size_t i = 0; i < 32; i++) if (!((token[i] >= '0' && token[i] <= '9') || (token[i] >= 'a' && token[i] <= 'f'))) return 0;
    return 1;
}
static int write_receipt(int fd, const char *token, const char *state) {
    char receipt[160]; int size = snprintf(receipt, sizeof(receipt), "%ld %s %s\n", (long)getpid(), token, state);
    return ftruncate(fd, 0) < 0 || pwrite(fd, receipt, (size_t)size, 0) != size || fsync(fd) < 0 ? -1 : 0;
}
static int is_worker_guard(pid_t pid, const char *token) {
    char path[80], command[4096]; snprintf(path, sizeof(path), "/proc/%ld/cmdline", (long)pid);
    int fd = open(path, O_RDONLY | O_CLOEXEC); if (fd < 0) return 0;
    ssize_t n = read(fd, command, sizeof(command) - 1); close(fd); if (n <= 0) return 0;
    command[n] = 0; int found_play = 0, found_token = 0;
    for (ssize_t pos = 0; pos < n; pos += (ssize_t)strlen(command + pos) + 1) {
        if (!strcmp(command + pos, "play")) found_play = 1;
        if (!strcmp(command + pos, token)) found_token = 1;
    }
    return found_play && found_token;
}
static int cancel_playback(const char *token) {
    if (!valid_token(token)) return 2;
    int fd = open(lock_path(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC); struct stat info;
    if (fd < 0 || fstat(fd, &info) < 0 || !S_ISREG(info.st_mode) || info.st_uid != geteuid() || info.st_mode & 077) { if (fd >= 0) close(fd); return 1; }
    char receipt[160] = {0}, recorded_token[40], state[40]; long pid;
    ssize_t size = pread(fd, receipt, sizeof(receipt) - 1, 0);
    if (size <= 0 || sscanf(receipt, "%ld %39s %39s", &pid, recorded_token, state) != 3 || strcmp(token, recorded_token)) { close(fd); return 1; }
    if (!flock(fd, LOCK_EX | LOCK_NB)) {
        close(fd); if (!strcmp(state, "CANCELLED")) { puts("CANCELLED"); return 0; } return 1;
    }
    if (!is_worker_guard((pid_t)pid, token) || kill((pid_t)pid, SIGTERM) < 0) { close(fd); return 1; }
    double deadline = monotonic_s() + 2;
    while (monotonic_s() < deadline) {
        if (!flock(fd, LOCK_EX | LOCK_NB)) {
            memset(receipt, 0, sizeof(receipt));
            size = pread(fd, receipt, sizeof(receipt) - 1, 0);
            int ok = size > 0 && sscanf(receipt, "%ld %39s %39s", &pid, recorded_token, state) == 3 && !strcmp(token, recorded_token) && !strcmp(state, "CANCELLED");
            close(fd); if (ok) { puts("CANCELLED"); return 0; } return 1;
        }
        struct timespec pause = {0, 10000000}; nanosleep(&pause, NULL);
    }
    close(fd); return 1;
}
int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOLBF, 0); signals();
    if (argc == 2 && !strcmp(argv[1], "status")) {
        printf("{\"device_present\":%s}\n", access("/dev/snd/pcmC0D0p", F_OK) == 0 ? "true" : "false"); return 0;
    }
    if (argc == 3 && !strcmp(argv[1], "cancel")) return cancel_playback(argv[2]);
    if ((argc != 3 && argc != 4) || strcmp(argv[1], "play")) { fputs("usage: robot-audio status | cancel TOKEN | play VOLUME [TOKEN] < prepared.wav\n", stderr); return 2; }
    const char *token = argc == 4 ? argv[3] : "manual";
    if (argc == 4 && !valid_token(token)) return 2;
    char *end; errno = 0; double volume = strtod(argv[2], &end);
    if (errno || *end || end == argv[2] || !isfinite(volume) || volume < 0 || volume > 1) return 2;
    unsigned char *buffer = NULL, *pcm = NULL; size_t frames = 0;
    if (read_audio(&buffer, &pcm, &frames) < 0) { puts(stopping ? "CANCELLED" : "INVALID_AUDIO"); return 2; }
    int lock = open(lock_path(), O_RDWR | O_CREAT | O_NOFOLLOW | O_CLOEXEC, 0600);
    struct stat info;
    if (lock < 0 || fstat(lock, &info) < 0 || !S_ISREG(info.st_mode) || info.st_uid != geteuid() || info.st_mode & 077) { free(buffer); if (lock >= 0) close(lock); return 2; }
    if (flock(lock, LOCK_EX | LOCK_NB) < 0) { puts("BUSY"); free(buffer); close(lock); return 3; }
    if (load_alsa() < 0) { puts("UNAVAILABLE"); free(buffer); close(lock); return 2; }
    struct snapshot *records = calloc(MAX_SNAPSHOTS, sizeof(*records));
    void *mixer = NULL; size_t count = 0;
    if (!records || snapshot_mixer(&mixer, records, &count) < 0) { free(records); free(buffer); if (mixer) snd_mixer_close(mixer); close(lock); return 2; }
    if (write_receipt(lock, token, "PLAYING") < 0) { snd_mixer_close(mixer); free(records); free(buffer); close(lock); return 2; }
    pid_t child = fork();
    if (child == 0) { int rc = play_worker(mixer, pcm, frames, volume); _exit(rc); }
    if (child < 0) { snd_mixer_close(mixer); free(records); free(buffer); close(lock); return 2; }
    printf("WORKER %ld\n", (long)child);
    int status = 0; double deadline = monotonic_s() + frames / 16000.0 + 3, stop_at = 0, heartbeat = monotonic_s();
    for (;;) {
        pid_t ended = waitpid(child, &status, WNOHANG);
        if (ended == child) break;
        if (ended < 0 && errno != EINTR) { status = 1 << 8; break; }
        double now = monotonic_s();
        if ((stopping || now > deadline) && !stop_at) { kill(child, SIGTERM); stop_at = now; }
        if (stop_at && now - stop_at > .3) kill(child, SIGKILL);
        if (now - heartbeat > .5) {
            if (puts("ALIVE") == EOF || fflush(stdout) == EOF) stopping = 1;
            heartbeat = now;
        }
        struct timespec pause = {0, 10000000}; nanosleep(&pause, NULL);
    }
    int (*handle_events)(void *) = NULL;
    *(void **)(&handle_events) = dlsym(alsa_library, "snd_mixer_handle_events");
    if (handle_events) handle_events(mixer);
    int restored = 1;
    int busy = WIFEXITED(status) && WEXITSTATUS(status) == 3;
    if (!busy) for (size_t i = 0; i < count; i++) if (set_value(&records[i]) < 0) restored = 0;
    snd_mixer_close(mixer); free(records); free(buffer);
    const char *result = !restored ? "RESTORE_FAILED" : busy ? "BUSY" : stopping ? "CANCELLED" : WIFEXITED(status) && WEXITSTATUS(status) == 0 ? "COMPLETED" : "ERROR";
    if (write_receipt(lock, token, result) < 0) result = "RESTORE_FAILED";
    close(lock); puts(result);
    if (!strcmp(result, "COMPLETED")) return 0;
    return stopping ? 130 : 1;
}
