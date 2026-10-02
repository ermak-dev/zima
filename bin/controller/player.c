#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <playerc.h>

static volatile sig_atomic_t interrupted;

static void on_signal(int signum)
{
    (void)signum;
    interrupted = 1;
}

static double now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int usage(void)
{
    fputs("usage: player <host> position|fan|stop|move <forward|backward|left|right> <seconds (0, 2]>\n", stderr);
    return 2;
}

int main(int argc, char **argv)
{
    double duration = 0, linear = 0, angular = 0;
    if (argc < 3 || !*argv[1]) {
        return usage();
    }
    if (!strcmp(argv[2], "move")) {
        if (argc != 5 || !*argv[4]) {
            return usage();
        }
        char *end;
        errno = 0;
        duration = strtod(argv[4], &end);
        if (errno || *end || !isfinite(duration) || duration <= 0 || duration > 2) {
            return usage();
        }
        if (!strcmp(argv[3], "forward")) linear = 0.1;
        else if (!strcmp(argv[3], "backward")) linear = -0.1;
        else if (!strcmp(argv[3], "left")) angular = 0.4;
        else if (!strcmp(argv[3], "right")) angular = -0.4;
        else return usage();
    } else if (argc != 3 || (strcmp(argv[2], "position") && strcmp(argv[2], "fan") && strcmp(argv[2], "stop"))) {
        return usage();
    }

    struct sigaction action = {.sa_handler = on_signal};
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    int result = 1, connected = 0, subscribed = 0, moving = 0;
    playerc_client_t *client = playerc_client_create(NULL, argv[1], 6665);
    playerc_position2d_t *base = NULL;
    playerc_ir_t *fan = NULL;
    if (!client) {
        fputs("player: cannot allocate client\n", stderr);
        return 1;
    }
    playerc_client_set_request_timeout(client, 3);
    if (playerc_client_connect(client)) {
        fprintf(stderr, "player: connect: %s\n", playerc_error_str());
        goto done;
    }
    connected = 1;
    if (!strcmp(argv[2], "fan")) {
        fan = playerc_ir_create(client, 6);
        if (!fan || playerc_ir_subscribe(fan, PLAYER_OPEN_MODE)) {
            fprintf(stderr, "player: fan subscribe: %s\n", playerc_error_str());
            goto done;
        }
    } else {
        base = playerc_position2d_create(client, 0);
        if (!base || playerc_position2d_subscribe(base, PLAYER_OPEN_MODE)) {
            fprintf(stderr, "player: subscribe: %s\n", playerc_error_str());
            goto done;
        }
    }
    subscribed = 1;

    if (!strcmp(argv[2], "fan")) {
        double deadline = now() + 3;
        while (!interrupted && now() < deadline) {
            int ready = playerc_client_peek(client, 250);
            if (ready < 0) break;
            if (ready && !playerc_client_read(client)) break;
            if (fan->info.datatime > 0) {
                printf("fan_motor_status: voltages=%u ranges=%u", fan->data.voltages_count, fan->data.ranges_count);
                for (uint32_t i = 0; i < fan->data.ranges_count; ++i) printf(" %.3f", fan->data.ranges[i]);
                putchar('\n');
                result = 0;
                break;
            }
        }
        if (result) fputs("player: no fresh fan motor telemetry within 3 seconds\n", stderr);
    } else if (!strcmp(argv[2], "position")) {
        double deadline = now() + 3;
        while (!interrupted && now() < deadline) {
            int ready = playerc_client_peek(client, 250);
            if (ready < 0) break;
            if (ready && !playerc_client_read(client)) break;
            if (base->info.datatime > 0) {
                printf("x=%.3f y=%.3f heading=%.3f vx=%.3f omega=%.3f\n",
                       base->px, base->py, base->pa, base->vx, base->va);
                result = 0;
                break;
            }
        }
        if (result) fputs("player: no fresh odometry within 3 seconds\n", stderr);
    } else if (!strcmp(argv[2], "stop")) {
        result = playerc_position2d_set_cmd_vel(base, 0, 0, 0, 1) ? 1 : 0;
    } else {
        double deadline = now() + duration;
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 100000000};
        while (!interrupted && now() < deadline) {
            if (playerc_position2d_set_cmd_vel(base, linear, 0, angular, 1)) {
                fprintf(stderr, "player: move: %s\n", playerc_error_str());
                break;
            }
            moving = 1;
            nanosleep(&pause, NULL);
        }
        if (!interrupted && now() >= deadline) result = 0;
    }

done:
    if (moving && playerc_position2d_set_cmd_vel(base, 0, 0, 0, 1)) {
        fputs("player: WARNING: failed to send zero velocity; stop robot locally\n", stderr);
        result = 1;
    }
    if (subscribed && fan) playerc_ir_unsubscribe(fan);
    if (subscribed && base) playerc_position2d_unsubscribe(base);
    if (fan) playerc_ir_destroy(fan);
    if (base) playerc_position2d_destroy(base);
    if (connected) playerc_client_disconnect(client);
    playerc_client_destroy(client);
    return result;
}
