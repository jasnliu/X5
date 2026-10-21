/* Strict per-image libjpeg decoder. No devices, files, stderr redirection or
 * global logging changes. Warning-producing/partial images are never accepted. */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <jpeglib.h>

struct guarded_error {
    struct jpeg_error_mgr base;
    jmp_buf jump;
    char reason[JMSG_LENGTH_MAX];
};
struct decode_state {
    struct jpeg_decompress_struct decoder;
    struct guarded_error error;
};

static void fail_decode(j_common_ptr info) {
    struct guarded_error *error = (struct guarded_error *)info->err;
    (*info->err->format_message)(info, error->reason);
    longjmp(error->jump, 1);
}

static void check_message(j_common_ptr info, int level) {
    /* libjpeg ordinarily prints a warning and recovers missing image data.
     * For metrology/calibration, a recovered partial image is not acceptable. */
    if (level < 0) fail_decode(info);
}

/* Returns 0=clean image, 1=corrupt JPEG, 2=wrong geometry, 3=local setup failure. */
int x5_decode_jpeg(const unsigned char *data, size_t size,
                   unsigned char *bgr, size_t capacity, int width, int height,
                   char *message, size_t message_capacity) {
    if (!message || !message_capacity) return 3;
    message[0] = '\0';
    if (!data || !size || !bgr || width <= 0 || height <= 0 || width > 8192 || height > 8192
            || capacity != (size_t)width * (size_t)height * 3) {
        snprintf(message, message_capacity, "Invalid decoder arguments");
        return 3;
    }
    /* Heap state stays defined across longjmp; no C++ destructors are skipped. */
    struct decode_state *state = calloc(1, sizeof(*state));
    if (!state) {
        snprintf(message, message_capacity, "Cannot allocate JPEG decoder");
        return 3;
    }
    state->decoder.err = jpeg_std_error(&state->error.base);
    state->error.base.error_exit = fail_decode;
    state->error.base.emit_message = check_message;
    state->error.base.output_message = fail_decode;
    if (setjmp(state->error.jump)) {
        snprintf(message, message_capacity, "%s", state->error.reason);
        jpeg_destroy_decompress(&state->decoder);
        free(state);
        return 1;
    }
    jpeg_create_decompress(&state->decoder);
    jpeg_mem_src(&state->decoder, data, size);
    jpeg_read_header(&state->decoder, TRUE);
    if (state->decoder.image_width != (JDIMENSION)width || state->decoder.image_height != (JDIMENSION)height) {
        snprintf(message, message_capacity, "JPEG dimensions %ux%u do not match %dx%d",
                 state->decoder.image_width, state->decoder.image_height, width, height);
        jpeg_destroy_decompress(&state->decoder);
        free(state);
        return 2;
    }
    state->decoder.out_color_space = JCS_EXT_BGR;
    jpeg_start_decompress(&state->decoder);
    if (state->decoder.output_width != (JDIMENSION)width
            || state->decoder.output_height != (JDIMENSION)height || state->decoder.output_components != 3) {
        snprintf(message, message_capacity, "Unexpected JPEG output geometry");
        jpeg_destroy_decompress(&state->decoder);
        free(state);
        return 2;
    }
    while (state->decoder.output_scanline < state->decoder.output_height) {
        JSAMPROW row = bgr + (size_t)state->decoder.output_scanline * (size_t)width * 3;
        if (jpeg_read_scanlines(&state->decoder, &row, 1) != 1) {
            snprintf(message, message_capacity, "Incomplete JPEG scanline");
            jpeg_destroy_decompress(&state->decoder);
            free(state);
            return 1;
        }
    }
    jpeg_finish_decompress(&state->decoder);
    jpeg_destroy_decompress(&state->decoder);
    free(state);
    return 0;
}
