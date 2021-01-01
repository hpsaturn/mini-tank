/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.4 */

#ifndef PB_COMM_PB_H_INCLUDED
#define PB_COMM_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _JoystickMessage {
    uint32_t ax;
    uint32_t ay;
    uint32_t az;
    uint32_t ck;
} JoystickMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define JoystickMessage_init_default             {0, 0, 0, 0}
#define JoystickMessage_init_zero                {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define JoystickMessage_ax_tag                   1
#define JoystickMessage_ay_tag                   2
#define JoystickMessage_az_tag                   3
#define JoystickMessage_ck_tag                   4

/* Struct field encoding specification for nanopb */
#define JoystickMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   ax,                1) \
X(a, STATIC,   REQUIRED, UINT32,   ay,                2) \
X(a, STATIC,   REQUIRED, UINT32,   az,                3) \
X(a, STATIC,   REQUIRED, UINT32,   ck,                4)
#define JoystickMessage_CALLBACK NULL
#define JoystickMessage_DEFAULT NULL

extern const pb_msgdesc_t JoystickMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define JoystickMessage_fields &JoystickMessage_msg

/* Maximum encoded size of messages (where known) */
#define JoystickMessage_size                     24

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif