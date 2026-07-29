// Thin JNI marshaling layer for com.example.dct3nokia.Dct3Engine. All real logic is
// in dct3_engine.c/.h (host-testable, no jni.h dependency) — this file only converts
// between Java/Kotlin and C types.

#include <jni.h>
#include <stdlib.h>

#include "dct3_engine.h"

JNIEXPORT jint JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeBoot(JNIEnv* env, jobject thiz, jstring jpath) {
    (void)thiz;
    const char* path = (*env)->GetStringUTFChars(env, jpath, NULL);
    if (!path) return -1;
    int rc = dct3and_boot(path);
    (*env)->ReleaseStringUTFChars(env, jpath, path);
    return (jint)rc;
}

JNIEXPORT void JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeRunCycles(JNIEnv* env, jobject thiz, jint cycles) {
    (void)env; (void)thiz;
    dct3and_run_cycles((int)cycles);
}

JNIEXPORT jint JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeLcdWidth(JNIEnv* env, jobject thiz) {
    (void)env; (void)thiz;
    return (jint)dct3and_lcd_width();
}

JNIEXPORT jint JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeLcdHeight(JNIEnv* env, jobject thiz) {
    (void)env; (void)thiz;
    return (jint)dct3and_lcd_height();
}

JNIEXPORT void JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeRenderPixels(JNIEnv* env, jobject thiz,
                                                          jintArray outPixels,
                                                          jint onArgb, jint offArgb) {
    (void)thiz;
    jsize len = (*env)->GetArrayLength(env, outPixels);
    jint* px = (*env)->GetIntArrayElements(env, outPixels, NULL);
    if (!px) return;
    dct3and_render_pixels((uint32_t*)px, (int)len, (uint32_t)onArgb, (uint32_t)offArgb);
    (*env)->ReleaseIntArrayElements(env, outPixels, px, 0);
}

JNIEXPORT jboolean JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeKeyEvent(JNIEnv* env, jobject thiz,
                                                      jint keyId, jboolean down) {
    (void)env; (void)thiz;
    return dct3and_key_event((int)keyId, down ? 1 : 0) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeReadAudio(JNIEnv* env, jobject thiz, jshortArray outSamples) {
    (void)thiz;
    jsize len = (*env)->GetArrayLength(env, outSamples);
    jshort* buf = (*env)->GetShortArrayElements(env, outSamples, NULL);
    if (!buf) return 0;
    int n = dct3and_read_audio((int16_t*)buf, (int)len);
    (*env)->ReleaseShortArrayElements(env, outSamples, buf, 0);
    return (jint)n;
}

JNIEXPORT jint JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeAudioRate(JNIEnv* env, jobject thiz) {
    (void)env; (void)thiz;
    return (jint)dct3and_audio_rate();
}

JNIEXPORT jboolean JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeIsPoweredOff(JNIEnv* env, jobject thiz) {
    (void)env; (void)thiz;
    return dct3and_is_powered_off() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeIsFaulted(JNIEnv* env, jobject thiz) {
    (void)env; (void)thiz;
    return dct3and_is_faulted() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_example_dct3nokia_Dct3Engine_nativeShutdown(JNIEnv* env, jobject thiz) {
    (void)env; (void)thiz;
    dct3and_shutdown();
}
