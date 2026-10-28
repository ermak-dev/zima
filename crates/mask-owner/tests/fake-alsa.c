
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
static const char *names[]={"speaker volume control","Audio speaker out","MIC1 boost AMP gain control"};
static long values[]={0,0,4};
static void logset(int id,long value){FILE *f=fopen(getenv("ROBOT_AUDIO_TEST_LOG"),"a");if(f){fprintf(f,"SET %s %ld\n",names[id],value);fclose(f);}}
int snd_pcm_open(void **p,const char*n,int stream,int mode){if(getenv("ROBOT_AUDIO_TEST_BUSY"))return -16;*p=(void*)1;return 0;}
int snd_pcm_close(void*p){return 0;}
int snd_pcm_set_params(void*p,int f,int a,unsigned c,unsigned r,int s,unsigned l){values[2]=5;return 0;}
long snd_pcm_writei(void*p,const void*b,unsigned long n){struct timespec t={0,20000000};nanosleep(&t,0);return n;}
int snd_pcm_state(void*p){return 3;}
int snd_pcm_start(void*p){return 0;}
int snd_pcm_nonblock(void*p,int n){return 0;}
int snd_pcm_drain(void*p){return 0;}
int snd_pcm_drop(void*p){return 0;}
const char *snd_strerror(int e){return "fake ALSA error";}
int snd_mixer_open(void**p,int m){*p=(void*)1;return 0;}
int snd_mixer_attach(void*p,const char*n){return 0;}
int snd_mixer_selem_register(void*p,void*a,void*b){return 0;}
int snd_mixer_load(void*p){return 0;}
int snd_mixer_close(void*p){return 0;}
void *snd_mixer_first_elem(void*p){return (void*)1;}
void *snd_mixer_elem_next(void*p){long n=(long)p;return n<3?(void*)(n+1):0;}
const char *snd_mixer_selem_get_name(void*p){return names[(long)p-1];}
int snd_mixer_selem_has_playback_volume(void*p){return (long)p!=2;}
int snd_mixer_selem_has_capture_volume(void*p){return 0;}
int snd_mixer_selem_has_playback_switch(void*p){return (long)p==2;}
int snd_mixer_selem_has_capture_switch(void*p){return 0;}
int snd_mixer_selem_is_enumerated(void*p){return 0;}
int snd_mixer_selem_get_playback_volume(void*p,int ch,long*v){if(ch)return -1;*v=values[(long)p-1];return 0;}
int snd_mixer_selem_get_capture_volume(void*p,int ch,long*v){return -1;}
int snd_mixer_selem_get_playback_switch(void*p,int ch,int*v){if(ch)return -1;*v=values[(long)p-1];return 0;}
int snd_mixer_selem_get_capture_switch(void*p,int ch,int*v){return -1;}
int snd_mixer_selem_get_enum_item(void*p,int ch,unsigned*v){return -1;}
int snd_mixer_selem_set_playback_volume(void*p,int ch,long v){values[(long)p-1]=v;logset((long)p-1,v);return 0;}
int snd_mixer_selem_set_capture_volume(void*p,int ch,long v){return 0;}
int snd_mixer_selem_set_playback_switch(void*p,int ch,int v){values[(long)p-1]=v;logset((long)p-1,v);return 0;}
int snd_mixer_selem_set_capture_switch(void*p,int ch,int v){return 0;}
int snd_mixer_selem_set_enum_item(void*p,int ch,unsigned v){return 0;}
int snd_mixer_selem_get_playback_volume_range(void*p,long*lo,long*hi){*lo=0;*hi=54;return 0;}
