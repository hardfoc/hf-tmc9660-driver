/**
 * @file tmc9660_param_mode_tmcl.hpp
 * @brief Enumerations and helpers for **TMCL® operation & reply codes** when the TMC9660 is
 *        used in *Parameter Mode*.
 *
 * Provides the TMCL command definitions, parameter enumerations, and helper structures needed
 * for complete motor control functionality: all operation codes, parameter definitions, and
 * reply structures.
 *
 * @defgroup TMC9660_TMCL TMCL Command Interface
 * @brief TMCL operation codes and command structures
 *
 * @defgroup TMC9660_TMCLParameters TMCL Parameters
 * @brief Parameter definitions for motor control and system configuration
 *
 * @defgroup TMC9660_TMCLReplies TMCL Reply Structures
 * @brief Reply structures and status codes for TMCL commands
 *
 * @defgroup TMC9660_TMCLTypes TMCL Type Definitions
 * @brief Enums and type definitions for TMCL operations
 *
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 * @copyright © 2025 Nebiyu Tadesse. Released under the GNU GPL V3 License.
 */

// clang-format off
#pragma once
#include <cstddef>
#include <cstdint>

/**
 * @brief TMCL command interface namespace for TMC9660.
 * @ingroup TMC9660_TMCL
 * 
 * This namespace contains all TMCL (Trinamic Motion Control Language) related
 * definitions for the TMC9660 motor driver. It provides comprehensive command
 * structures, parameter definitions, and reply handling for parameter mode
 * operation of the TMC9660.
 */
namespace tmc9660::tmcl {


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ████████╗███╗   ███╗ ██████╗██╗          █████╗ ██████╗ ██╗                                                   //
//    ╚══██╔══╝████╗ ████║██╔════╝██║         ██╔══██╗██╔══██╗██║                                                   //
//       ██║   ██╔████╔██║██║     ██║         ███████║██████╔╝██║                                                   //
//       ██║   ██║╚██╔╝██║██║     ██║         ██╔══██║██╔═══╝ ██║                                                   //
//       ██║   ██║ ╚═╝ ██║╚██████╗███████╗    ██║  ██║██║     ██║                                                   //
//       ╚═╝   ╚═╝     ╚═╝ ╚═════╝╚══════╝    ╚═╝  ╚═╝╚═╝     ╚═╝                                                   //
//                                                                                                                  //
//==================================================================================================================//
//                                                  TMCL API SECTION                                                //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  TMCL operation codes (Table 18)
//--------------------------------------
/**
 * @brief Operation codes accepted by the TMC9660 Parameter Mode (Table 18).
 *
 * Table 18 — TMCL commands:
 *  NUMBER | OPERATION      | TYPE                | MOTOR/BANK         | VALUE                | DESCRIPTION
 *  ------ | -------------- | ------------------- | ------------------ | -------------------- | ---------------------------------------------------------------
 *     3   | MST            | -                   | -                  | -                    | Stops motor movement
 *     5   | SAP            | parameter           | 0                  | value                | Sets axis parameter (motion control specific settings)
 *     6   | GAP            | parameter           | 0                  | -                    | Gets axis parameter (read out motion control specific settings)
 *     7   | STAP           | 0xFFF               | 0xF                | 0xFFFFFFFF           | Stores all storable parameters to external memory
 *     9   | SGP            | parameter           | 0,2,3              | value                | Sets global parameter (module/user settings)
 *    10   | GGP            | parameter           | -                  | -                    | Gets global parameter (module/user settings)
 *    13   | RFS            | START|STOP|STATUS   | 0                  | -                    | Reference search
 *    14   | SIO            | port number         | 0                  | 0,1                  | Sets digital output to specified value
 *    15   | GIO            | port number         | 0: digital, 1: ana | -                    | Gets value of analog/digital input
 *    19   | CALC           | operation           | -                  | value                | Arithmetic operation (accumulator and value)
 *    20   | COMP           | -                   | -                  | value                | Compares accumulator with value
 *    21   | JC             | condition           | -                  | address              | Jump conditional
 *    22   | JA             | -                   | -                  | address              | Jump absolute
 *    23   | CSUB           | -                   | -                  | address              | Calls subroutine
 *    24   | RSUB           | -                   | -                  | -                    | Returns from subroutine
 *    25   | EI             | -                   | -                  | interrupt number     | Enables interrupt
 *    26   | DI             | -                   | -                  | interrupt number     | Disables interrupt
 *    27   | WAIT           | condition           | -                  | ticks                | Waits with further program execution
 *    28   | STOP           | -                   | -                  | -                    | Stops program execution
 *    33   | CALCX          | type                | -                  | -                    | Arithmetic operation (accumulator and X-register)
 *    34   | AAP            | parameter           | 0                  | -                    | Accumulator to axis parameter
 *    35   | AGP            | parameter           | 0,2,3              | -                    | Accumulator to global parameter
 *    36   | CLE            | flag                | -                  | -                    | Clears an error flag
 *    37   | VECT           | interrupt number    | -                  | address              | Defines interrupt vector
 *    38   | RETI           | -                   | -                  | -                    | Returns from interrupt
 *    40   | CALCVV         | type                | user var 1         | user var 2           | Arithmetic: user variable and user variable
 *    41   | CALCVA         | type                | user variable      | -                    | Arithmetic: user variable and accumulator
 *    42   | CALCAV         | type                | user variable      | -                    | Arithmetic: accumulator and user variable
 *    43   | CALCVX         | type                | user variable      | -                    | Arithmetic: user variable and X register
 *    44   | CALCXV         | type                | user variable      | -                    | Arithmetic: X register and user variable
 *    45   | CALCV          | type                | -                  | value                | Arithmetic: user variable and direct value
 *    48   | RST            | -                   | -                  | address              | Restarts the program from the given address
 *    49   | DJNZ           | user variable       | -                  | address              | Decrement and jump if not zero
 *    55   | SIV            | -                   | -                  | value                | Sets indexed variable
 *    56   | GIV            | -                   | -                  | -                    | Gets indexed variable
 *    57   | AIV            | -                   | -                  | -                    | Accumulator to indexed variable
 *   128   | ApplStop        | -                  | -                  | -                    | Stops a running TMCL program
 *   129   | ApplRun         | 0: current, 1: addr| -                  | address              | Starts/continues TMCL program
 *   130   | ApplStep        | -                  | -                  | -                    | Executes only the next TMCL command
 *   131   | ApplReset       | -                  | -                  | -                    | Stops and resets program counter
 *   132   | DownloadStart   | -                  | -                  | -                    | Enter download mode (store script)
 *   133   | DownloadEnd     | -                  | -                  | -                    | End download mode
 *   134   | ReadMem         | -                  | -                  | address              | Returns script command at address
 *   135   | GetStatusScript | -                  | -                  | -                    | Gets status of script
 *   136   | GetVersion      | -                  | -                  | -                    | Gets version of system
 *   137   | FactoryDefault  | -                  | -                  | -                    | Clears settings stored in external memory
 *   141   | Breakpoint      | 0:add,1:del,2:all,3:max| -              | address              | Manage breakpoints
 *   142   | RamDebug        | see Table 16       | see Table 16       | see Table 16         | Access to RAMDebug control
 *   157   | GetInfo         | 0:ID, 1:Version    | -                  | -                    | Gets ID and version info
 *   242   | Boot            | 0x81               | 0x92               | 0xA3B4C5D6           | Returns the system to the bootloader
 *
 * Each command lists the *TYPE*, *MOTOR/BANK*, and *VALUE* fields as per Table 18.
 * For scripting-related commands, see the manual for further details.
 */
#define OP_LIST(X) \
    X(MST,             3,   /*!< Stop motor movement. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(SAP,             5,   /*!< Set Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: value. */) \
    X(GAP,             6,   /*!< Get Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: -. */) \
    X(STAP,            7,   /*!< Store All Parameters. TYPE: 0xFFF, MOTOR/BANK: 0xF, VALUE: 0xFFFFFFFF. */) \
    X(SGP,             9,   /*!< Set Global Parameter. TYPE: parameter, MOTOR/BANK: 0,2,3, VALUE: value. */) \
    X(GGP,             10,  /*!< Get Global Parameter. TYPE: parameter, MOTOR/BANK: -, VALUE: -. */) \
    X(RFS,             13,  /*!< Reference Search. TYPE: START|STOP|STATUS, MOTOR/BANK: 0, VALUE: -. */) \
    X(SIO,             14,  /*!< Set IO. TYPE: port number, MOTOR/BANK: 0, VALUE: 0,1. */) \
    X(GIO,             15,  /*!< Get IO. TYPE: port number, MOTOR/BANK: 0 (digital) or 1 (analog), VALUE: -. */) \
    X(CALC,            19,  /*!< Arithmetic operation. TYPE: operation, MOTOR/BANK: -, VALUE: value. */) \
    X(COMP,            20,  /*!< Compare accumulator. TYPE: -, MOTOR/BANK: -, VALUE: value. */) \
    X(JC,              21,  /*!< Jump Conditional. TYPE: condition, MOTOR/BANK: -, VALUE: address. */) \
    X(JA,              22,  /*!< Jump Absolute. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(CSUB,            23,  /*!< Call Subroutine. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(RSUB,            24,  /*!< Return from Subroutine. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(EI,              25,  /*!< Enable Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: interrupt number. */) \
    X(DI,              26,  /*!< Disable Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: interrupt number. */) \
    X(WAIT,            27,  /*!< Wait. TYPE: condition, MOTOR/BANK: -, VALUE: ticks. */) \
    X(STOP,            28,  /*!< Stop Script Execution. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(CALCX,           33,  /*!< Arithmetic accumulator <-> X-register. TYPE: type, MOTOR/BANK: -, VALUE: -. */) \
    X(AAP,             34,  /*!< Accumulator to Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: -. */) \
    X(AGP,             35,  /*!< Accumulator to Global Parameter. TYPE: parameter, MOTOR/BANK: 0,2,3, VALUE: -. */) \
    X(CLE,             36,  /*!< Clear Error Flag. TYPE: flag, MOTOR/BANK: -, VALUE: -. */) \
    X(VECT,            37,  /*!< Define Interrupt Vector. TYPE: interrupt number, MOTOR/BANK: -, VALUE: address. */) \
    X(RETI,            38,  /*!< Return from Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(CALCVV,          40,  /*!< UserVar ∘ UserVar arithmetic. TYPE: type, MOTOR/BANK: user variable 1, VALUE: user variable 2. */) \
    X(CALCVA,          41,  /*!< UserVar ∘ Accumulator. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCAV,          42,  /*!< Accumulator ∘ UserVar. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCVX,          43,  /*!< UserVar ∘ X-register. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCXV,          44,  /*!< X-register ∘ UserVar. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCV,           45,  /*!< UserVar ∘ literal value. TYPE: type, MOTOR/BANK: -, VALUE: value. */) \
    X(RST,             48,  /*!< Restart script from address. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(DJNZ,            49,  /*!< Decrement-and-Jump if not zero. TYPE: user variable, MOTOR/BANK: -, VALUE: address. */) \
    X(SIV,             55,  /*!< Set Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: value. */) \
    X(GIV,             56,  /*!< Get Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(AIV,             57,  /*!< Accumulator to Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ApplStop,        128, /*!< Stop running TMCL program. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ApplRun,         129, /*!< Run/continue TMCL program. TYPE: 0 (current addr) or 1 (specified), MOTOR/BANK: -, VALUE: address. */) \
    X(ApplStep,        130, /*!< Execute single TMCL instruction. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ApplReset,       131, /*!< Reset program counter. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(DownloadStart,   132, /*!< Enter download (script upload) mode. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(DownloadEnd,     133, /*!< Leave download mode. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ReadMem,         134, /*!< Read script word at address. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(GetStatusScript, 135, /*!< Get script status. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(GetVersion,      136, /*!< Get firmware version string. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(FactoryDefault,  137, /*!< Erase stored config & reset. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(Breakpoint,      141, /*!< Manage breakpoints. TYPE: 0 (add), 1 (del), 2 (del all), 3 (get max), MOTOR/BANK: -, VALUE: address. */) \
    X(RamDebug,        142, /*!< RAMDebug Control. TYPE/MOTOR/BANK/VALUE: see Table 16. */) \
    X(GetInfo,         157, /*!< Generic info (ID, version, etc.). TYPE: 0 (ID), 1 (Version), MOTOR/BANK: -, VALUE: -. */) \
    X(Boot,            242, /*!< Exit to bootloader. TYPE: 0x81, MOTOR/BANK: 0x92, VALUE: 0xA3B4C5D6. */)

enum class Op : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    OP_LIST(X)
    #undef X
};

inline const char* to_string(Op op) {
    switch(op) {
        #define X(NAME, VALUE, DOC) case Op::NAME: return #NAME;
        OP_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef OP_LIST

//--------------------------------------
//  Arithmetic operation types for CALC
//--------------------------------------
#define CALC_OPERATION_LIST(X) \
    X(CALC_ADD, 0, /*!< Addition */) \
    X(CALC_SUB, 1, /*!< Subtract */) \
    X(CALC_MUL, 2, /*!< Multiply */) \
    X(CALC_DIV, 3, /*!< Divide */) \
    X(CALC_MOD, 4, /*!< Modulo */) \
    X(CALC_AND, 5, /*!< Bitwise AND */) \
    X(CALC_OR,  6, /*!< Bitwise OR */) \
    X(CALC_XOR, 7, /*!< Bitwise XOR */) \
    X(CALC_NOT, 8, /*!< Bitwise NOT */) \
    X(CALC_LOAD,9, /*!< Load/replace */)

enum class CalcOperation : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE,
    CALC_OPERATION_LIST(X)
    #undef X
};

inline const char* to_string(CalcOperation op) {
    switch(op) {
        #define X(NAME, VALUE, DOC) case CalcOperation::NAME: return #NAME;
        CALC_OPERATION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef CALC_OPERATION_LIST

//--------------------------------------
//  Conditional jump conditions for JC
//--------------------------------------
#define JUMP_CONDITION_LIST(X) \
    X(JC_ZERO, 0, /*!< Accumulator == 0 */) \
    X(JC_NOT_ZERO, 1, /*!< Accumulator != 0 */) \
    X(JC_EQUAL, 2, /*!< Result equal */) \
    X(JC_NOT_EQUAL, 3, /*!< Result not equal */) \
    X(JC_GREATER, 4, /*!< Greater than */) \
    X(JC_GREATER_EQ, 5, /*!< Greater or equal */) \
    X(JC_LOWER, 6, /*!< Lower than */) \
    X(JC_LOWER_EQ, 7, /*!< Lower or equal */) \
    X(JC_TIMEOUT, 8, /*!< WAIT timeout flag */)

enum class JumpCondition : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE,
    JUMP_CONDITION_LIST(X)
    #undef X
};

inline const char* to_string(JumpCondition c) {
    switch(c) {
        #define X(NAME, VALUE, DOC) case JumpCondition::NAME: return #NAME;
        JUMP_CONDITION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef JUMP_CONDITION_LIST

//--------------------------------------
//  WAIT event conditions
//--------------------------------------
#define WAIT_CONDITION_LIST(X) \
    X(WAIT_TIME, 0, /*!< Fixed time */) \
    X(WAIT_RFS, 4, /*!< Reference search finished */) \
    X(WAIT_LATCH, 5, /*!< Latch event */) \
    X(WAIT_VEL_REACHED, 6, /*!< Velocity reached */) \
    X(WAIT_POS_REACHED, 7, /*!< Position reached */) \
    X(WAIT_STOP_LEFT, 8, /*!< Left stop switch */) \
    X(WAIT_STOP_RIGHT, 9, /*!< Right stop switch */) \
    X(WAIT_STOP_HOME, 10, /*!< Home switch */)

enum class WaitCondition : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE,
    WAIT_CONDITION_LIST(X)
    #undef X
};

inline const char* to_string(WaitCondition w) {
    switch(w) {
        #define X(NAME, VALUE, DOC) case WaitCondition::NAME: return #NAME;
        WAIT_CONDITION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef WAIT_CONDITION_LIST

//--------------------------------------
//  TMCL reply codes (Table 19)
//--------------------------------------
/**
 * @brief Status codes returned in the *status* byte of every TMCL reply.
 *
 * Table 19 — TMCL status codes:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | ---------------------------------------------------------------
 *   100   | REPLY_OK                    | Command executed successfully.
 *   101   | REPLY_CMD_LOADED            | Command loaded successfully.
 *     1   | REPLY_CHKERR                | Check error occurred (e.g., checksum error).
 *     2   | REPLY_INVALID_CMD           | Invalid command received (unknown command number).
 *     3   | REPLY_WRONG_TYPE            | Wrong type of data received (TYPE field invalid).
 *     4   | REPLY_INVALID_VALUE         | Invalid value received (VALUE field out of range).
 *     6   | REPLY_CMD_NOT_AVAILABLE     | Command not available (not supported in this mode).
 *     7   | REPLY_CMD_LOAD_ERROR        | Error occurred while loading command (storage error).
 *     9   | REPLY_MAX_EXCEEDED          | Maximum limit exceeded (e.g., too many breakpoints).
 *    10   | REPLY_DOWNLOAD_NOT_POSSIBLE | Download operation not possible (misuse or memory full).
 */
#define REPLY_CODE_LIST(X) \
    X(REPLY_OK,                     100, /*!< Command executed successfully. */) \
    X(REPLY_CMD_LOADED,             101, /*!< Command loaded successfully. */) \
    X(REPLY_CHKERR,                   1, /*!< Check error occurred (e.g., checksum error). */) \
    X(REPLY_INVALID_CMD,              2, /*!< Invalid command received (unknown command number). */) \
    X(REPLY_WRONG_TYPE,               3, /*!< Wrong type of data received (TYPE field invalid). */) \
    X(REPLY_INVALID_VALUE,            4, /*!< Invalid value received (VALUE field out of range). */) \
    X(REPLY_CMD_NOT_AVAILABLE,        6, /*!< Command not available (not supported in this mode). */) \
    X(REPLY_CMD_LOAD_ERROR,           7, /*!< Error occurred while loading command (storage error). */) \
    X(REPLY_MAX_EXCEEDED,             9, /*!< Maximum limit exceeded (e.g., too many breakpoints). */) \
    X(REPLY_DOWNLOAD_NOT_POSSIBLE,   10, /*!< Download operation not possible (misuse or memory full). */)

enum class ReplyCode : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REPLY_CODE_LIST(X)
    #undef X
};

inline const char* to_string(ReplyCode rc) {
    switch(rc) {
        #define X(NAME, VALUE, DOC) case ReplyCode::NAME: return #NAME;
        REPLY_CODE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REPLY_CODE_LIST

//--------------------------------------
//  RAMDebug sub‑commands / type codes (Table 16)
//--------------------------------------
/**
 * @brief TYPE field values when **Operation::RamDebug** is issued.
 *
 * Table 16 — RAMDebug type commands:
 *  NUMBER | TYPE (subcode)      | MOTOR/BANK                                      | VALUE                                                                 | DESCRIPTION
 *  ------ | ------------------- | ----------------------------------------------- | --------------------------------------------------------------------- | ----------------------------------------------------------
 *     0   | INITIALISE_RESET    | -                                               | -                                                                     | Initialize and reset RAMDebug configuration & buffers.
 *     1   | SET_SAMPLE_COUNT    | -                                               | Number                                                                | Set total number of samples to collect (not per-channel).
 *     3   | SET_PRESCALER       | -                                               | Prescale                                                              | Set divider for sampling rate (divider = VALUE+1).
 *     4   | SET_CHANNEL         | Type: 0=Disabled, 1=Parameter, 3=Global param   | Motor/Bank: 0xFF000000, AP/GP number: 0x00000FFF                      | Configure capture channel.
 *     5   | SET_TRIGGER_CHANNEL | Type: 0=Disabled, 1=Parameter, 3=Global param   | Motor/Bank: 0xFF000000, AP/GP number: 0x00000FFF                      | Specify source of trigger data.
 *     6   | SET_TRIGGER_MASK_SHIFT | Shift                                       | Mask                                                                  | Specify mask and shift for trigger value.
 *     7   | ENABLE_TRIGGER      | Type: 0=Uncond, 1=RiseS, 2=FallS, 3=DualS,      | Threshold                                                             | Start measurement by enabling trigger.
 *         |                     | 4=RiseU, 5=FallU, 6=BothU                       |                                                                       |
 *     8   | GET_STATE           | -                                               | -                                                                     | Request state of RAMDebug (see RamDebugState).
 *     9   | READ_SAMPLE         | Index                                           | -                                                                     | Download sampled values at index.
 *    10   | GET_INFO            | -                                               | 0=Max channels, 1=Buffer size, 2=RAMDebug freq, 3=Captured count,     | Read general info.
 *         |                     |                                                 | 4=Prescaler on trigger                                                |
 *    11   | GET_CHANNEL_TYPE    | Index                                           | -                                                                     | Read channel type info.
 *    12   | GET_CHANNEL_ADDRESS | Index                                           | -                                                                     | Read channel address.
 *    13   | SET_PRETRIGGER_COUNT | -                                              | Number                                                                | Set total number of pretrigger samples (not per-channel).
 *    14   | GET_PRETRIGGER_COUNT | -                                              | -                                                                     | Get total number of pretrigger samples.
 */
#define RAMDEBUG_TYPE_LIST(X) \
    X(INITIALISE_RESET,        0,  /*!< Initialize and reset RAMDebug configuration & buffers. */) \
    X(SET_SAMPLE_COUNT,        1,  /*!< VALUE: Number of samples to collect in total (not per-channel). */) \
    X(SET_PRESCALER,           3,  /*!< VALUE: Prescale value. Sets divider for sampling rate (divider = VALUE+1). */) \
    X(SET_CHANNEL,             4,  /*!< Configure capture channel. TYPE: 0=Disabled, 1=Parameter, 3=Global parameter. MOTOR/BANK: 0xFF000000. VALUE: AP/GP number (0x00000FFF). */) \
    X(SET_TRIGGER_CHANNEL,     5,  /*!< Specify trigger source. TYPE: 0=Disabled, 1=Parameter, 3=Global parameter. MOTOR/BANK: 0xFF000000. VALUE: AP/GP number (0x00000FFF). */) \
    X(SET_TRIGGER_MASK_SHIFT,  6,  /*!< Specify mask and shift for trigger value. MOTOR/BANK: Shift. VALUE: Mask. */) \
    X(ENABLE_TRIGGER,          7,  /*!< Start measurement by enabling trigger. TYPE: 0=Unconditional, 1=Rising edge signed, 2=Falling edge signed, 3=Dual edge signed, 4=Rising edge unsigned, 5=Falling edge unsigned, 6=Both edge unsigned. VALUE: Threshold. */) \
    X(GET_STATE,               8,  /*!< Request state of RAMDebug (see RamDebugState). */) \
    X(READ_SAMPLE,             9,  /*!< Download sampled values. MOTOR/BANK: Index. */) \
    X(GET_INFO,               10,  /*!< Read general info. VALUE: 0=Max channels, 1=Buffer size, 2=RAMDebug frequency, 3=Captured sample count, 4=Prescaler value on trigger event. */) \
    X(GET_CHANNEL_TYPE,       11,  /*!< Read channel type info. MOTOR/BANK: Index. */) \
    X(GET_CHANNEL_ADDRESS,    12,  /*!< Read channel address. MOTOR/BANK: Index. */) \
    X(SET_PRETRIGGER_COUNT,   13,  /*!< Set total number of pretrigger samples (not per-channel). VALUE: Number of samples. */) \
    X(GET_PRETRIGGER_COUNT,   14,  /*!< Get total number of pretrigger samples. */)

enum class RamDebugType : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    RAMDEBUG_TYPE_LIST(X)
    #undef X
};

inline const char* to_string(RamDebugType t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case RamDebugType::NAME: return #NAME;
        RAMDEBUG_TYPE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef RAMDEBUG_TYPE_LIST

//--------------------------------------
//  RAMDebug state machine (Table 17)
//--------------------------------------
/**
 * @brief Values returned by ::RamDebugType::GET_STATE.
 *
 * Table 17 — List of RAMDebug states:
 *  NUMBER | NAME       | DESCRIPTION
 *  ------ | ---------- | -------------------------------------------------------------------------------------------
 *     0   | Idle       | RAMDebug is not running and can be configured. Use type code 0 to enter this state.
 *     1   | Trigger    | RAMDebug is waiting for the trigger event to happen. When updating a value that RAMDebug
 *                      | is triggering on, ensure this state is reached before updating.
 *     2   | Capture    | RAMDebug has been triggered and is capturing samples.
 *     3   | Complete   | RAMDebug has finished capturing samples. The data can now be downloaded using type code 9.
 *     4   | Pretrigger | RAMDebug is capturing samples for the pretrigger.
 */
#define RAMDEBUG_STATE_LIST(X) \
    X(IDLE, 0,       /*!< Idle: RAMDebug is not running and can be configured. Use type code 0 to enter this state. */) \
    X(TRIGGER, 1,    /*!< Trigger: Waiting for the trigger event. Ensure this state before updating trigger value. */) \
    X(CAPTURE, 2,    /*!< Capture: RAMDebug has been triggered and is capturing samples. */) \
    X(COMPLETE, 3,   /*!< Complete: Finished capturing samples. Data can be downloaded using type code 9. */) \
    X(PRETRIGGER, 4, /*!< Pretrigger: Capturing samples for the pretrigger. */)

enum class RamDebugState : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    RAMDEBUG_STATE_LIST(X)
    #undef X
};

inline const char* to_string(RamDebugState s) {
    switch(s) {
        #define X(NAME, VALUE, DOC) case RamDebugState::NAME: return #NAME;
        RAMDEBUG_STATE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef RAMDEBUG_STATE_LIST


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//     ██████╗ ██╗      ██████╗ ██████╗  █████╗ ██╗         ██████╗  █████╗ ██████╗  █████╗ ███╗   ███╗███████╗     //
//    ██╔════╝ ██║     ██╔═══██╗██╔══██╗██╔══██╗██║         ██╔══██╗██╔══██╗██╔══██╗██╔══██╗████╗ ████║██╔════╝     //
//    ██║  ███╗██║     ██║   ██║██████╔╝███████║██║         ██████╔╝███████║██████╔╝███████║██╔████╔██║███████╗     //
//    ██║   ██║██║     ██║   ██║██╔══██╗██╔══██║██║         ██╔═══╝ ██╔══██║██╔══██╗██╔══██║██║╚██╔╝██║╚════██║     //
//    ╚██████╔╝███████╗╚██████╔╝██████╔╝██║  ██║███████╗    ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║███████║     //
//     ╚═════╝ ╚══════╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝    ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝     //
//                                                                                                                  //
//==================================================================================================================//
//                                            GLOBAL PARAMETERS SECTION                                             //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Global Parameters – Bank 0 (System Settings)
//--------------------------------------
/**
 * @brief Non-motion parameters in bank 0: communication, I/O, heartbeat, hibernation, loops, auto-start, etc.
 *
 * To persist changes, use STAP after setting RWE parameters.
 */
#define GLOBAL_PARAM_BANK0_LIST(X) \
    X(SERIAL_ADDRESS,                          1,  /*!< RS485/UART module address [1…255 odd]. Default: 1. RWE */) \
    X(SERIAL_HOST_ADDRESS,                     2,  /*!< RS485/UART host address [1…255]. Default: 2. RWE */) \
    X(HEARTBEAT_MONITORING_CONFIG,             3,  /*!< 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. Default: 0. RWE */) \
    X(HEARTBEAT_MONITORING_TIMEOUT,            4,  /*!< Heartbeat timeout [ms] [1…4294967295]. Default: 100. RWE */) \
    X(IO_DIRECTION_MASK,                       5,  /*!< GPIO direction mask [bit=1→output]. Default: 0. RWE */) \
    X(IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK,    6,  /*!< GPIO pull-enable mask [bit=1→pull enabled]. Default: 0. RWE */) \
    X(IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK, 7,  /*!< GPIO pull-dir mask [bit=1→pull-up]. Default: 0. RWE */) \
    X(WAKE_PIN_CONTROL_ENABLE,                10,  /*!< 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(GO_TO_TIMEOUT_POWER_DOWN_STATE,         11,  /*!< See PowerDownTimeout. Default: 0. W */) \
    X(MAIN_LOOPS,                             12,  /*!< Main loops/sec [0…4294967295]. Default: 0. R */) \
    X(TORQUE_LOOPS,                           13,  /*!< Torque loops/sec [0…4294967295]. Default: 0. R */) \
    X(VELOCITY_LOOPS,                         14,  /*!< Velocity loops/sec [0…4294967295]. Default: 0. R */) \
    X(AUTO_START_ENABLE,                      77,  /*!< 0: DISABLED, 1: ENABLED. Default: 1. RWE */) \
    X(CLEAR_USER_VARIABLES,                   85,  /*!< 0: TRY_LOAD_FROM_STORAGE, 1: CLEAR. Default: 0. RWE */)

enum class GlobalParamBank0 : std::uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
        GLOBAL_PARAM_BANK0_LIST(X)
    #undef X
};

inline const char* to_string(GlobalParamBank0 p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case GlobalParamBank0::NAME: return #NAME;
            GLOBAL_PARAM_BANK0_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GLOBAL_PARAM_BANK0_LIST

//--------------------------------------
//  Global Parameters – Bank 2 (User Variables)
//--------------------------------------
/**
 * @brief User-script variables 0…15. RWE.
 */
#define GLOBAL_PARAM_BANK2_LIST(X) \
    X(USER_VARIABLE_0,  0,  /*!< User-script variable 0. */) \
    X(USER_VARIABLE_1,  1,  /*!< User-script variable 1. */) \
    X(USER_VARIABLE_2,  2,  /*!< User-script variable 2. */) \
    X(USER_VARIABLE_3,  3,  /*!< User-script variable 3. */) \
    X(USER_VARIABLE_4,  4,  /*!< User-script variable 4. */) \
    X(USER_VARIABLE_5,  5,  /*!< User-script variable 5. */) \
    X(USER_VARIABLE_6,  6,  /*!< User-script variable 6. */) \
    X(USER_VARIABLE_7,  7,  /*!< User-script variable 7. */) \
    X(USER_VARIABLE_8,  8,  /*!< User-script variable 8. */) \
    X(USER_VARIABLE_9,  9,  /*!< User-script variable 9. */) \
    X(USER_VARIABLE_10, 10, /*!< User-script variable 10. */) \
    X(USER_VARIABLE_11, 11, /*!< User-script variable 11. */) \
    X(USER_VARIABLE_12, 12, /*!< User-script variable 12. */) \
    X(USER_VARIABLE_13, 13, /*!< User-script variable 13. */) \
    X(USER_VARIABLE_14, 14, /*!< User-script variable 14. */) \
    X(USER_VARIABLE_15, 15, /*!< User-script variable 15. */)

enum class GlobalParamBank2 : std::uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
        GLOBAL_PARAM_BANK2_LIST(X)
    #undef X
};

inline const char* to_string(GlobalParamBank2 p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case GlobalParamBank2::NAME: return #NAME;
            GLOBAL_PARAM_BANK2_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GLOBAL_PARAM_BANK2_LIST

//--------------------------------------
//  Global Parameters – Bank 3 (Interrupt & Trigger Configuration)
//--------------------------------------
/**
 * @brief Timer periods and input-trigger transitions for scripting interrupts.
 */
#define GLOBAL_PARAM_BANK3_LIST(X) \
    X(TIMER_0_PERIOD, 0,   /*!< [ms] 0…2147483647. R/W */) \
    X(TIMER_1_PERIOD, 1,   /*!< [ms] 0…2147483647. R/W */) \
    X(TIMER_2_PERIOD, 2,   /*!< [ms] 0…2147483647. R/W */) \
    X(STOP_LEFT_TRIGGER_TRANSITION, 10,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(STOP_RIGHT_TRIGGER_TRANSITION, 11, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(HOME_TRIGGER_TRANSITION, 12,       /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_0_TRIGGER_TRANSITION, 13,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_1_TRIGGER_TRANSITION, 14,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_2_TRIGGER_TRANSITION, 15,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_3_TRIGGER_TRANSITION, 16,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_4_TRIGGER_TRANSITION, 17,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_5_TRIGGER_TRANSITION, 18,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_6_TRIGGER_TRANSITION, 19,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_7_TRIGGER_TRANSITION, 20,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_8_TRIGGER_TRANSITION, 21,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_9_TRIGGER_TRANSITION, 22,  /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_10_TRIGGER_TRANSITION, 23, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_11_TRIGGER_TRANSITION, 24, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_12_TRIGGER_TRANSITION, 25, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_13_TRIGGER_TRANSITION, 26, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_14_TRIGGER_TRANSITION, 27, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_15_TRIGGER_TRANSITION, 28, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_16_TRIGGER_TRANSITION, 29, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_17_TRIGGER_TRANSITION, 30, /*!< See TriggerTransition. Default: 0. R/W */) \
    X(INPUT_18_TRIGGER_TRANSITION, 31, /*!< See TriggerTransition. Default: 0. R/W */)

enum class GlobalParamBank3 : std::uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
        GLOBAL_PARAM_BANK3_LIST(X)
    #undef X
};

inline const char* to_string(GlobalParamBank3 p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case GlobalParamBank3::NAME: return #NAME;
            GLOBAL_PARAM_BANK3_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GLOBAL_PARAM_BANK3_LIST

/**
 * @brief Compile-time bank of each global-parameter enum (SGP/GGP MOTOR/BANK field).
 *
 * Global parameters live in banks 0, 2 and 3 (reference manual Tables 43–45); axis
 * parameters (::Parameters) use SAP/GAP and are never addressed with a bank. Driver APIs that
 * take one of these enums deduce the bank from the type, so a GlobalParamBank0 id cannot be
 * sent with bank 2, and an axis id cannot be sent with SGP/GGP at all.
 */
template <typename E> inline constexpr bool kIsGlobalParamBank = false;
template <> inline constexpr bool kIsGlobalParamBank<GlobalParamBank0> = true;
template <> inline constexpr bool kIsGlobalParamBank<GlobalParamBank2> = true;
template <> inline constexpr bool kIsGlobalParamBank<GlobalParamBank3> = true;

/// Concept form of kIsGlobalParamBank.
template <typename E>
concept IsGlobalParamBank = kIsGlobalParamBank<E>;

/// Bank number of a global-parameter enum type.
template <typename E> inline constexpr std::uint8_t kGlobalBankOf = 0xFF;
template <> inline constexpr std::uint8_t kGlobalBankOf<GlobalParamBank0> = 0;
template <> inline constexpr std::uint8_t kGlobalBankOf<GlobalParamBank2> = 2;
template <> inline constexpr std::uint8_t kGlobalBankOf<GlobalParamBank3> = 3;

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Trigger Transition Options
/// @{
//--------------------------------------
//  Trigger Transition Options
//--------------------------------------
/**
 * @brief For all "_TRIGGER_TRANSITION" params: 0=OFF, 1=RISING, 2=FALLING, 3=BOTH.
 * 
 * NUMBER | NAME    | DESCRIPTION
 * ------ | ------- | -----------
 * 0      | OFF     | No trigger transition.
 * 1      | RISING  | Trigger on rising edge.
 * 2      | FALLING | Trigger on falling edge.
 * 3      | BOTH    | Trigger on both rising and falling edges.
 */
#define TRIGGER_TRANSITION_LIST(X) \
    X(OFF,     0, /*!< No trigger transition. */) \
    X(RISING,  1, /*!< Trigger on rising edge. */) \
    X(FALLING, 2, /*!< Trigger on falling edge. */) \
    X(BOTH,    3, /*!< Trigger on both rising and falling edges. */)

enum class TriggerTransition : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    TRIGGER_TRANSITION_LIST(X)
    #undef X
};

inline const char* to_string(TriggerTransition e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case TriggerTransition::NAME: return #NAME;
        TRIGGER_TRANSITION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef TRIGGER_TRANSITION_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//                              ██████╗  █████╗ ██████╗  █████╗ ███╗   ███╗███████╗                                 //
//                              ██╔══██╗██╔══██╗██╔══██╗██╔══██╗████╗ ████║██╔════╝                                 //
//                              ██████╔╝███████║██████╔╝███████║██╔████╔██║███████╗                                 //
//                              ██╔═══╝ ██╔══██║██╔══██╗██╔══██║██║╚██╔╝██║╚════██║                                 //
//                              ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║███████║                                 //
//                              ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝                                 //
//                                                                                                                  //
//==================================================================================================================//
//                                                  PARAMETERS                                                      //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗  █████╗ ████████╗███████╗    ██████╗ ██████╗ ██╗██╗   ██╗███████╗██████╗ ███████╗                     //
//   ██╔════╝ ██╔══██╗╚══██╔══╝██╔════╝    ██╔══██╗██╔══██╗██║██║   ██║██╔════╝██╔══██╗██╔════╝                     //
//   ██║  ███╗███████║   ██║   █████╗      ██║  ██║██████╔╝██║██║   ██║█████╗  ██████╔╝███████╗                     //
//   ██║   ██║██╔══██║   ██║   ██╔══╝      ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝  ██╔══██╗╚════██║                     //
//   ╚██████╔╝██║  ██║   ██║   ███████╗    ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗██║  ██║███████║                     //
//    ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝╚═╝  ╚═╝╚══════╝                     //
//                                                                                                                  //
//==================================================================================================================//
//                                              GATE DRIVER SECTION                                                 //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Gate Driver Parameters (Table 20)
//--------------------------------------
/**
 * @brief Parameters for gate driver timer and current settings (Table 20).
 *
 * These parameters configure the MOSFET gate driver for the TMC9660, including output polarity, break-before-make timing,
 * adaptive drive time, drive times, current limits, and bootstrap current limit.
 *
 * Table 20 — Gate driver timer and current settings:
 *  NR   | PARAMETER                        | DESCRIPTION
 *  ---- | -------------------------------- | ---------------------------------------------------------------------------
 *  233  | PWM_L_OUTPUT_POLARITY            | PWM_L output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW
 *  234  | PWM_H_OUTPUT_POLARITY            | PWM_H output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW
 *  235  | BREAK_BEFORE_MAKE_TIME_LOW_UVW   | Break-before-make time for low side UVW [8.33ns units] 0...255
 *  236  | BREAK_BEFORE_MAKE_TIME_HIGH_UVW  | Break-before-make time for high side UVW [8.33ns units] 0...255
 *  237  | BREAK_BEFORE_MAKE_TIME_LOW_Y2    | Break-before-make time for low side Y2 [8.33ns units] 0...255
 *  238  | BREAK_BEFORE_MAKE_TIME_HIGH_Y2   | Break-before-make time for high side Y2 [8.33ns units] 0...255
 *  239  | USE_ADAPTIVE_DRIVE_TIME_UVW      | Adaptive drive time UVW. 0: DISABLED, 1: ENABLED
 *  240  | USE_ADAPTIVE_DRIVE_TIME_Y2       | Adaptive drive time Y2. 0: DISABLED, 1: ENABLED
 *  241  | DRIVE_TIME_SINK_UVW              | Discharge time UVW [0...255] (1s/120MHz) × (2×value+3)
 *  242  | DRIVE_TIME_SOURCE_UVW            | Charge time UVW [0...255] (1s/120MHz) × (2×value+3)
 *  243  | DRIVE_TIME_SINK_Y2               | Discharge time Y2 [0...255] (1s/120MHz) × (2×value+3)
 *  244  | DRIVE_TIME_SOURCE_Y2             | Charge time Y2 [0...255] (1s/120MHz) × (2×value+3)
 *  245  | UVW_SINK_CURRENT                 | See ::tmc9660::tmcl::GateCurrentSink
 *  246  | UVW_SOURCE_CURRENT               | See ::tmc9660::tmcl::GateCurrentSource
 *  247  | Y2_SINK_CURRENT                  | See ::tmc9660::tmcl::GateCurrentSink
 *  248  | Y2_SOURCE_CURRENT                | See ::tmc9660::tmcl::GateCurrentSource
 *  249  | BOOTSTRAP_CURRENT_LIMIT          | See ::tmc9660::tmcl::BootstrapCurrentLimit
 */
#define GATE_DRIVER_LIST(X) \
    X(PWM_L_OUTPUT_POLARITY,           233, /*!< PWM_L output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. */) \
    X(PWM_H_OUTPUT_POLARITY,           234, /*!< PWM_H output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. */) \
    X(BREAK_BEFORE_MAKE_TIME_LOW_UVW,  235, /*!< Break-before-make time for low side UVW [8.33ns units] 0...255. */) \
    X(BREAK_BEFORE_MAKE_TIME_HIGH_UVW, 236, /*!< Break-before-make time for high side UVW [8.33ns units] 0...255. */) \
    X(BREAK_BEFORE_MAKE_TIME_LOW_Y2,   237, /*!< Break-before-make time for low side Y2 [8.33ns units] 0...255. */) \
    X(BREAK_BEFORE_MAKE_TIME_HIGH_Y2,  238, /*!< Break-before-make time for high side Y2 [8.33ns units] 0...255. */) \
    X(USE_ADAPTIVE_DRIVE_TIME_UVW,     239, /*!< Adaptive drive time UVW. 0: DISABLED, 1: ENABLED. */) \
    X(USE_ADAPTIVE_DRIVE_TIME_Y2,      240, /*!< Adaptive drive time Y2. 0: DISABLED, 1: ENABLED. */) \
    X(DRIVE_TIME_SINK_UVW,             241, /*!< Discharge time UVW [0...255] (1s/120MHz) × (2×value+3). */) \
    X(DRIVE_TIME_SOURCE_UVW,           242, /*!< Charge time UVW [0...255] (1s/120MHz) × (2×value+3). */) \
    X(DRIVE_TIME_SINK_Y2,              243, /*!< Discharge time Y2 [0...255] (1s/120MHz) × (2×value+3). */) \
    X(DRIVE_TIME_SOURCE_Y2,            244, /*!< Charge time Y2 [0...255] (1s/120MHz) × (2×value+3). */) \
    X(UVW_SINK_CURRENT,                245, /*!< See GateCurrentSink. */) \
    X(UVW_SOURCE_CURRENT,              246, /*!< See GateCurrentSource. */) \
    X(Y2_SINK_CURRENT,                 247, /*!< See GateCurrentSink. */) \
    X(Y2_SOURCE_CURRENT,               248, /*!< See GateCurrentSource. */) \
    X(BOOTSTRAP_CURRENT_LIMIT,         249, /*!< See BootstrapCurrentLimit. */)

enum class GateDriver : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GATE_DRIVER_LIST(X)
    #undef X
};

inline const char* to_string(GateDriver gd) {
    switch (gd) {
        #define X(NAME, VALUE, DOC) case GateDriver::NAME: return #NAME;
        GATE_DRIVER_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}


/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////
//--------------------------------------
//  PWM Output Polarity (Table)
//--------------------------------------
/**
 * @brief 
 *
 * Table — PWM Output Polarity:
 *  NUMBER | NAME          | DESCRIPTION
 *  ------ | ------------- | -----------------------------------------------------------------
 *     0   | ACTIVE_HIGH   | PWM output is active high.
 *     1   | ACTIVE_LOW    | PWM output is active low.
 */
#define PWM_OUTPUT_POLARITY_LIST(X) \
    X(ACTIVE_HIGH, 0, /*!< PWM output is active high. */) \
    X(ACTIVE_LOW,  1, /*!< PWM output is active low. */)

enum class PwmOutputPolarity : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    PWM_OUTPUT_POLARITY_LIST(X)
    #undef X
};

inline const char* to_string(PwmOutputPolarity p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case PwmOutputPolarity::NAME: return #NAME;
        PWM_OUTPUT_POLARITY_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef PWM_OUTPUT_POLARITY_LIST

//--------------------------------------
//  Gate Current Sink (Table)
//--------------------------------------
/**
 * @brief Enumerates possible values for UVW_SINK_CURRENT and Y2_SINK_CURRENT (50–2000mA).
 *
 * Table — Gate Current Sink:
 *  NUMBER | NAME       | DESCRIPTION
 *  ------ | ---------- | -----------------------------------------------------------------
 *     0   | CUR_50_MA  | 50 mA
 *     1   | CUR_100_MA | 100 mA
 *     2   | CUR_160_MA | 160 mA
 *     3   | CUR_210_MA | 210 mA
 *     4   | CUR_270_MA | 270 mA
 *     5   | CUR_320_MA | 320 mA
 *     6   | CUR_380_MA | 380 mA
 *     7   | CUR_430_MA | 430 mA
 *     8   | CUR_580_MA | 580 mA
 *     9   | CUR_720_MA | 720 mA
 *    10   | CUR_860_MA | 860 mA
 *    11   | CUR_1000_MA| 1000 mA
 *    12   | CUR_1250_MA| 1250 mA
 *    13   | CUR_1510_MA| 1510 mA
 *    14   | CUR_1770_MA| 1770 mA
 *    15   | CUR_2000_MA| 2000 mA
 */
#define GATE_CURRENT_SINK_LIST(X) \
    X(CUR_50_MA,    0,  /*!< 50 mA */) \
    X(CUR_100_MA,   1,  /*!< 100 mA */) \
    X(CUR_160_MA,   2,  /*!< 160 mA */) \
    X(CUR_210_MA,   3,  /*!< 210 mA */) \
    X(CUR_270_MA,   4,  /*!< 270 mA */) \
    X(CUR_320_MA,   5,  /*!< 320 mA */) \
    X(CUR_380_MA,   6,  /*!< 380 mA */) \
    X(CUR_430_MA,   7,  /*!< 430 mA */) \
    X(CUR_580_MA,   8,  /*!< 580 mA */) \
    X(CUR_720_MA,   9,  /*!< 720 mA */) \
    X(CUR_860_MA,   10, /*!< 860 mA */) \
    X(CUR_1000_MA,  11, /*!< 1000 mA */) \
    X(CUR_1250_MA,  12, /*!< 1250 mA */) \
    X(CUR_1510_MA,  13, /*!< 1510 mA */) \
    X(CUR_1770_MA,  14, /*!< 1770 mA */) \
    X(CUR_2000_MA,  15, /*!< 2000 mA */)

enum class GateCurrentSink : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GATE_CURRENT_SINK_LIST(X)
    #undef X
};

inline const char* to_string(GateCurrentSink s) {
    switch(s) {
        #define X(NAME, VALUE, DOC) case GateCurrentSink::NAME: return #NAME;
        GATE_CURRENT_SINK_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GATE_CURRENT_SINK_LIST

//--------------------------------------
//  Gate Current Source (Table)
//--------------------------------------
/**
 * @brief Enumerates possible values for UVW_SOURCE_CURRENT and Y2_SOURCE_CURRENT (25–1000mA).
 *
 * Table — Gate Current Source:
 *  NUMBER | NAME       | DESCRIPTION
 *  ------ | ---------- | -----------------------------------------------------------------
 *     0   | CUR_25_MA  | 25 mA
 *     1   | CUR_50_MA  | 50 mA
 *     2   | CUR_80_MA  | 80 mA
 *     3   | CUR_105_MA | 105 mA
 *     4   | CUR_135_MA | 135 mA
 *     5   | CUR_160_MA | 160 mA
 *     6   | CUR_190_MA | 190 mA
 *     7   | CUR_215_MA | 215 mA
 *     8   | CUR_290_MA | 290 mA
 *     9   | CUR_360_MA | 360 mA
 *    10   | CUR_430_MA | 430 mA
 *    11   | CUR_500_MA | 500 mA
 *    12   | CUR_625_MA | 625 mA
 *    13   | CUR_755_MA | 755 mA
 *    14   | CUR_855_MA | 855 mA
 *    15   | CUR_1000_MA| 1000 mA
 */
#define GATE_CURRENT_SOURCE_LIST(X) \
    X(CUR_25_MA,    0,  /*!< 25 mA */) \
    X(CUR_50_MA,    1,  /*!< 50 mA */) \
    X(CUR_80_MA,    2,  /*!< 80 mA */) \
    X(CUR_105_MA,   3,  /*!< 105 mA */) \
    X(CUR_135_MA,   4,  /*!< 135 mA */) \
    X(CUR_160_MA,   5,  /*!< 160 mA */) \
    X(CUR_190_MA,   6,  /*!< 190 mA */) \
    X(CUR_215_MA,   7,  /*!< 215 mA */) \
    X(CUR_290_MA,   8,  /*!< 290 mA */) \
    X(CUR_360_MA,   9,  /*!< 360 mA */) \
    X(CUR_430_MA,   10, /*!< 430 mA */) \
    X(CUR_500_MA,   11, /*!< 500 mA */) \
    X(CUR_625_MA,   12, /*!< 625 mA */) \
    X(CUR_755_MA,   13, /*!< 755 mA */) \
    X(CUR_855_MA,   14, /*!< 855 mA */) \
    X(CUR_1000_MA,  15, /*!< 1000 mA */)

enum class GateCurrentSource : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GATE_CURRENT_SOURCE_LIST(X)
    #undef X
};

inline const char* to_string(GateCurrentSource s) {
    switch(s) {
        #define X(NAME, VALUE, DOC) case GateCurrentSource::NAME: return #NAME;
        GATE_CURRENT_SOURCE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GATE_CURRENT_SOURCE_LIST

//--------------------------------------
//  Bootstrap Current Limit (Table)
//--------------------------------------
/**
 * @brief Enumerates possible values for BOOTSTRAP_CURRENT_LIMIT (45–391mA).
 *
 * Table — Bootstrap Current Limit:
 *  NUMBER | NAME       | DESCRIPTION
 *  ------ | ---------- | -----------------------------------------------------------------
 *     0   | CUR_45_MA  | 45 mA
 *     1   | CUR_91_MA  | 91 mA
 *     2   | CUR_141_MA | 141 mA
 *     3   | CUR_191_MA | 191 mA
 *     4   | CUR_267_MA | 267 mA
 *     5   | CUR_292_MA | 292 mA
 *     6   | CUR_341_MA | 341 mA
 *     7   | CUR_391_MA | 391 mA
 */
#define BOOTSTRAP_CURRENT_LIMIT_LIST(X) \
    X(CUR_45_MA,   0,  /*!< 45 mA */) \
    X(CUR_91_MA,   1,  /*!< 91 mA */) \
    X(CUR_141_MA,  2,  /*!< 141 mA */) \
    X(CUR_191_MA,  3,  /*!< 191 mA */) \
    X(CUR_267_MA,  4,  /*!< 267 mA */) \
    X(CUR_292_MA,  5,  /*!< 292 mA */) \
    X(CUR_341_MA,  6,  /*!< 341 mA */) \
    X(CUR_391_MA,  7,  /*!< 391 mA */)

enum class BootstrapCurrentLimit : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    BOOTSTRAP_CURRENT_LIMIT_LIST(X)
    #undef X
};

inline const char* to_string(BootstrapCurrentLimit s) {
    switch(s) {
        #define X(NAME, VALUE, DOC) case BootstrapCurrentLimit::NAME: return #NAME;
        BOOTSTRAP_CURRENT_LIMIT_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef BOOTSTRAP_CURRENT_LIMIT_LIST

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Gate Driver Overcurrent Protection
//--------------------------------------
/**
 * @brief Parameters for configuring gate driver overcurrent protection.
 * 
 * These parameters configure the overcurrent protection thresholds, blanking times,
 * deglitch times, and enable/disable settings for the UVW and Y2 phases of the TMC9660 gate driver.
 *
 * Table — Gate Driver Overcurrent Protection:
 *  NUMBER | NAME                     | DESCRIPTION
 *  ------ | ------------------------ | ---------------------------------------------------------------------------
 *    254  | UVW_LOW_SIDE_ENABLE      | Enable overcurrent protection for UVW phases low side. 0: DISABLED, 1: ENABLED. Default: 1.
 *    255  | UVW_HIGH_SIDE_ENABLE     | Enable overcurrent protection for UVW phases high side. 0: DISABLED, 1: ENABLED. Default: 1.
 *    256  | Y2_LOW_SIDE_ENABLE       | Enable overcurrent protection for Y2 phase low side. 0: DISABLED, 1: ENABLED. Default: 1.
 *    257  | Y2_HIGH_SIDE_ENABLE      | Enable overcurrent protection for Y2 phase high side. 0: DISABLED, 1: ENABLED. Default: 1.
 *    258  | UVW_LOW_SIDE_THRESHOLD   | Threshold for UVW phases low side [0-15]. Default: 0.
 *    259  | UVW_HIGH_SIDE_THRESHOLD  | Threshold for UVW phases high side [0-15]. Default: 0.
 *    260  | Y2_LOW_SIDE_THRESHOLD    | Threshold for Y2 phase low side [0-15]. Default: 0.
 *    261  | Y2_HIGH_SIDE_THRESHOLD   | Threshold for Y2 phase high side [0-15]. Default: 0.
 *    262  | UVW_LOW_SIDE_BLANKING    | Blanking time for UVW phases low side [0-7]. Default: 2.
 *    263  | UVW_HIGH_SIDE_BLANKING   | Blanking time for UVW phases high side [0-7]. Default: 2.
 *    264  | Y2_LOW_SIDE_BLANKING     | Blanking time for Y2 phase low side [0-7]. Default: 2.
 *    265  | Y2_HIGH_SIDE_BLANKING    | Blanking time for Y2 phase high side [0-7]. Default: 2.
 *    266  | UVW_LOW_SIDE_DEGLITCH    | Deglitch time for UVW phases low side [0-7]. Default: 6.
 *    267  | UVW_HIGH_SIDE_DEGLITCH   | Deglitch time for UVW phases high side [0-7]. Default: 6.
 *    268  | Y2_LOW_SIDE_DEGLITCH     | Deglitch time for Y2 phase low side [0-7]. Default: 6.
 *    269  | Y2_HIGH_SIDE_DEGLITCH    | Deglitch time for Y2 phase high side [0-7]. Default: 6.
 *    270  | UVW_LOW_SIDE_USE_VDS     | Use VDS measurement for UVW phases low side. 0: DISABLED, 1: ENABLED. Default: 1.
 *    271  | Y2_LOW_SIDE_USE_VDS      | Use VDS measurement for Y2 phase low side. 0: DISABLED, 1: ENABLED. Default: 1.
 */
#define OVERCURRENT_PROTECTION_LIST(X) \
    X(UVW_LOW_SIDE_ENABLE,     254, /*!< Enable overcurrent protection for UVW phases low side. 0: DISABLED, 1: ENABLED. Default: 1. */) \
    X(UVW_HIGH_SIDE_ENABLE,    255, /*!< Enable overcurrent protection for UVW phases high side. 0: DISABLED, 1: ENABLED. Default: 1. */) \
    X(Y2_LOW_SIDE_ENABLE,      256, /*!< Enable overcurrent protection for Y2 phase low side. 0: DISABLED, 1: ENABLED. Default: 1. */) \
    X(Y2_HIGH_SIDE_ENABLE,     257, /*!< Enable overcurrent protection for Y2 phase high side. 0: DISABLED, 1: ENABLED. Default: 1. */) \
    X(UVW_LOW_SIDE_THRESHOLD,  258, /*!< Threshold for UVW phases low side [0-15]. Default: 0. */) \
    X(UVW_HIGH_SIDE_THRESHOLD, 259, /*!< Threshold for UVW phases high side [0-15]. Default: 0. */) \
    X(Y2_LOW_SIDE_THRESHOLD,   260, /*!< Threshold for Y2 phase low side [0-15]. Default: 0. */) \
    X(Y2_HIGH_SIDE_THRESHOLD,  261, /*!< Threshold for Y2 phase high side [0-15]. Default: 0. */) \
    X(UVW_LOW_SIDE_BLANKING,   262, /*!< Blanking time for UVW phases low side [0-7]. Default: 2. */) \
    X(UVW_HIGH_SIDE_BLANKING,  263, /*!< Blanking time for UVW phases high side [0-7]. Default: 2. */) \
    X(Y2_LOW_SIDE_BLANKING,    264, /*!< Blanking time for Y2 phase low side [0-7]. Default: 2. */) \
    X(Y2_HIGH_SIDE_BLANKING,   265, /*!< Blanking time for Y2 phase high side [0-7]. Default: 2. */) \
    X(UVW_LOW_SIDE_DEGLITCH,   266, /*!< Deglitch time for UVW phases low side [0-7]. Default: 6. */) \
    X(UVW_HIGH_SIDE_DEGLITCH,  267, /*!< Deglitch time for UVW phases high side [0-7]. Default: 6. */) \
    X(Y2_LOW_SIDE_DEGLITCH,    268, /*!< Deglitch time for Y2 phase low side [0-7]. Default: 6. */) \
    X(Y2_HIGH_SIDE_DEGLITCH,   269, /*!< Deglitch time for Y2 phase high side [0-7]. Default: 6. */) \
    X(UVW_LOW_SIDE_USE_VDS,    270, /*!< Use VDS measurement for UVW phases low side. 0: DISABLED, 1: ENABLED. Default: 1. */) \
    X(Y2_LOW_SIDE_USE_VDS,     271, /*!< Use VDS measurement for Y2 phase low side. 0: DISABLED, 1: ENABLED. Default: 1. */)

enum class OvercurrentProtection : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    OVERCURRENT_PROTECTION_LIST(X)
    #undef X
};

inline const char* to_string(OvercurrentProtection p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case OvercurrentProtection::NAME: return #NAME;
        OVERCURRENT_PROTECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  Overcurrent Protection Enable/Disable
//--------------------------------------
/**
 * @brief Enumerates options for enabling or disabling overcurrent protection.
 *
 * Table — Overcurrent Protection Enable/Disable:
 *  NUMBER | NAME      | DESCRIPTION
 *  ------ | --------- | -----------------------------------------------------------------
 *     0   | DISABLED  | Protection disabled.
 *     1   | ENABLED   | Protection enabled.
 */
#define OVERCURRENT_ENABLE_LIST(X) \
    X(DISABLED, 0, /*!< Protection disabled. */) \
    X(ENABLED,  1, /*!< Protection enabled. */)

enum class OvercurrentEnable : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    OVERCURRENT_ENABLE_LIST(X)
    #undef X
};

inline const char* to_string(OvercurrentEnable e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case OvercurrentEnable::NAME: return #NAME;
        OVERCURRENT_ENABLE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef OVERCURRENT_ENABLE_LIST

//--------------------------------------
//  Overcurrent Protection Thresholds
//--------------------------------------
/**
 * @brief Enumerates possible threshold values for overcurrent protection.
 *
 * **Important:** The hardware automatically selects the appropriate threshold based on the sensing method:
 * - **High-Side (HS)**: Always uses VDS sensing, thresholds are VDS values only
 * - **Low-Side (LS)**: Threshold depends on UVW_LOW_SIDE_USE_VDS / Y2_LOW_SIDE_USE_VDS setting:
 *   - If VDS enabled: Uses VDS threshold values (63mV, 125mV, 187mV, etc.)
 *   - If RSHUNT enabled: Uses RSHUNT threshold values (80mV, 165mV, 250mV, etc.)
 *
 * The enum names reflect the dual nature for low-side thresholds (e.g., V_80_OR_63_MILLIVOLT means
 * 80mV for RSHUNT or 63mV for VDS). For high-side, only the VDS value applies.
 *
 * Table — Overcurrent Protection Thresholds (per datasheet Table 11):
 *  NUMBER | NAME                    | HS_VDS | LS_RSHUNT | LS_VDS
 *  ------ | ----------------------- | ------ | --------- | ------
 *     0   | V_80_OR_63_MILLIVOLT    | 63 mV  | 80 mV     | 63 mV
 *     1   | V_165_OR_125_MILLIVOLT  | 125 mV | 165 mV    | 125 mV
 *     2   | V_250_OR_187_MILLIVOLT  | 187 mV | 250 mV    | 187 mV
 *     3   | V_330_OR_248_MILLIVOLT  | 248 mV | 330 mV    | 248 mV
 *     4   | V_415_OR_312_MILLIVOLT | 312 mV | 415 mV    | 312 mV
 *     5   | V_500_OR_374_MILLIVOLT | 374 mV | 500 mV    | 374 mV
 *     6   | V_582_OR_434_MILLIVOLT | 434 mV | 582 mV    | 434 mV
 *     7   | V_660_OR_504_MILLIVOLT | 504 mV | 660 mV    | 504 mV
 *     8   | V_125_OR_705_MILLIVOLT | 705 mV | 125 mV    | 705 mV
 *     9   | V_250_OR_940_MILLIVOLT | 940 mV | 250 mV    | 940 mV
 *    10   | V_375_OR_1180_MILLIVOLT| 1180 mV| 375 mV    | 1180 mV
 *    11   | V_500_OR_1410_MILLIVOLT| 1410 mV| 500 mV    | 1410 mV
 *    12   | V_625_OR_1650_MILLIVOLT| 1650 mV| 625 mV    | 1650 mV
 *    13   | V_750_OR_1880_MILLIVOLT| 1880 mV| 750 mV    | 1880 mV
 *    14   | V_875_OR_2110_MILLIVOLT| 2110 mV| 875 mV    | 2110 mV
 *    15   | V_1000_OR_2350_MILLIVOLT| 2350 mV| 1000 mV   | 2350 mV
 */
#define OVERCURRENT_THRESHOLD_LIST(X) \
    X(V_80_OR_63_MILLIVOLT,    0, /*!< HS: 63mV, LS_RSHUNT: 80mV, LS_VDS: 63mV */) \
    X(V_165_OR_125_MILLIVOLT,   1, /*!< HS: 125mV, LS_RSHUNT: 165mV, LS_VDS: 125mV */) \
    X(V_250_OR_187_MILLIVOLT,   2, /*!< HS: 187mV, LS_RSHUNT: 250mV, LS_VDS: 187mV */) \
    X(V_330_OR_248_MILLIVOLT,   3, /*!< HS: 248mV, LS_RSHUNT: 330mV, LS_VDS: 248mV */) \
    X(V_415_OR_312_MILLIVOLT,   4, /*!< HS: 312mV, LS_RSHUNT: 415mV, LS_VDS: 312mV */) \
    X(V_500_OR_374_MILLIVOLT,   5, /*!< HS: 374mV, LS_RSHUNT: 500mV, LS_VDS: 374mV */) \
    X(V_582_OR_434_MILLIVOLT,   6, /*!< HS: 434mV, LS_RSHUNT: 582mV, LS_VDS: 434mV */) \
    X(V_660_OR_504_MILLIVOLT,   7, /*!< HS: 504mV, LS_RSHUNT: 660mV, LS_VDS: 504mV */) \
    X(V_125_OR_705_MILLIVOLT,   8, /*!< HS: 705mV, LS_RSHUNT: 125mV, LS_VDS: 705mV */) \
    X(V_250_OR_940_MILLIVOLT,   9, /*!< HS: 940mV, LS_RSHUNT: 250mV, LS_VDS: 940mV */) \
    X(V_375_OR_1180_MILLIVOLT, 10, /*!< HS: 1180mV, LS_RSHUNT: 375mV, LS_VDS: 1180mV */) \
    X(V_500_OR_1410_MILLIVOLT, 11, /*!< HS: 1410mV, LS_RSHUNT: 500mV, LS_VDS: 1410mV */) \
    X(V_625_OR_1650_MILLIVOLT, 12, /*!< HS: 1650mV, LS_RSHUNT: 625mV, LS_VDS: 1650mV */) \
    X(V_750_OR_1880_MILLIVOLT, 13, /*!< HS: 1880mV, LS_RSHUNT: 750mV, LS_VDS: 1880mV */) \
    X(V_875_OR_2110_MILLIVOLT, 14, /*!< HS: 2110mV, LS_RSHUNT: 875mV, LS_VDS: 2110mV */) \
    X(V_1000_OR_2350_MILLIVOLT, 15, /*!< HS: 2350mV, LS_RSHUNT: 1000mV, LS_VDS: 2350mV */)

enum class OvercurrentThreshold : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    OVERCURRENT_THRESHOLD_LIST(X)
    #undef X
};

inline const char* to_string(OvercurrentThreshold t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case OvercurrentThreshold::NAME: return #NAME;
        OVERCURRENT_THRESHOLD_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef OVERCURRENT_THRESHOLD_LIST

//--------------------------------------
//  Overcurrent Protection Blanking and Deglitch Times
//--------------------------------------
/**
 * @brief Enumerates possible blanking and deglitch times for overcurrent protection.
 *
 * Table — Overcurrent Protection Timing:
 *  NUMBER | NAME             | DESCRIPTION
 *  ------ | ---------------- | -----------------------------------------------------------------
 *     0   | OFF              | No blanking or deglitching
 *     1   | T_0_25_MICROSEC  | 0.25 µs
 *     2   | T_0_5_MICROSEC   | 0.5 µs
 *     3   | T_1_MICROSEC     | 1.0 µs
 *     4   | T_2_MICROSEC     | 2.0 µs
 *     5   | T_4_MICROSEC     | 4.0 µs
 *     6   | T_6_MICROSEC     | 6.0 µs
 *     7   | T_8_MICROSEC     | 8.0 µs
 */
#define OVERCURRENT_TIMING_LIST(X) \
    X(OFF,              0, /*!< No blanking or deglitching */) \
    X(T_0_25_MICROSEC,  1, /*!< 0.25 µs */) \
    X(T_0_5_MICROSEC,   2, /*!< 0.5 µs */) \
    X(T_1_MICROSEC,     3, /*!< 1.0 µs */) \
    X(T_2_MICROSEC,     4, /*!< 2.0 µs */) \
    X(T_4_MICROSEC,     5, /*!< 4.0 µs */) \
    X(T_6_MICROSEC,     6, /*!< 6.0 µs */) \
    X(T_8_MICROSEC,     7, /*!< 8.0 µs */)

enum class OvercurrentTiming : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    OVERCURRENT_TIMING_LIST(X)
    #undef X
};

inline const char* to_string(OvercurrentTiming t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case OvercurrentTiming::NAME: return #NAME;
        OVERCURRENT_TIMING_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef OVERCURRENT_TIMING_LIST

//--------------------------------------
//  VDS Measurement Enable/Disable
//--------------------------------------
/**
 * @brief Enumerates options for VDS measurement usage in overcurrent protection.
 *
 * Table — VDS Measurement Enable/Disable:
 *  NUMBER | NAME      | DESCRIPTION
 *  ------ | --------- | -----------------------------------------------------------------
 *     0   | DISABLED  | VDS measurement disabled
 *     1   | ENABLED   | VDS measurement enabled
 */
#define VDS_USAGE_LIST(X) \
    X(DISABLED, 0, /*!< VDS measurement disabled */) \
    X(ENABLED,  1, /*!< VDS measurement enabled */)

enum class VdsUsage : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VDS_USAGE_LIST(X)
    #undef X
};

inline const char* to_string(VdsUsage v) {
    switch(v) {
        #define X(NAME, VALUE, DOC) case VdsUsage::NAME: return #NAME;
        VDS_USAGE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VDS_USAGE_LIST

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Undervoltage Protection Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring undervoltage lockout (UVLO) protection.
 *
 * These parameters configure the undervoltage protection for the supply voltage (VS),
 * driver voltage (VDRV), and bootstrap capacitors for UVW and Y2 phases.
 *
 * Table — Undervoltage Protection Parameters:
 *  NUMBER | NAME             | DESCRIPTION
 *  ------ | ---------------- | -----------------------------------------------------------------
 *    250  | SUPPLY_LEVEL     | Protection level for VS (Supply voltage) [0-16]. Default: 0.
 *    251  | VDRV_ENABLE      | Enable protection for VDRV (Driver voltage) [0, 1]. Default: 1.
 *    252  | BST_UVW_ENABLE   | Enable protection for UVW bootstrap capacitors [0, 1]. Default: 1.
 *    253  | BST_Y2_ENABLE    | Enable protection for Y2 bootstrap capacitor [0, 1]. Default: 1.
 */
#define UNDERVOLTAGE_PROTECTION_LIST(X) \
    X(SUPPLY_LEVEL,     250, /*!< Protection level for VS (Supply voltage) [0-16]. Default: 0. */) \
    X(VDRV_ENABLE,      251, /*!< Enable protection for VDRV (Driver voltage) [0, 1]. Default: 1. */) \
    X(BST_UVW_ENABLE,   252, /*!< Enable protection for UVW bootstrap capacitors [0, 1]. Default: 1. */) \
    X(BST_Y2_ENABLE,    253, /*!< Enable protection for Y2 bootstrap capacitor [0, 1]. Default: 1. */)

enum class UndervoltageProtection : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    UNDERVOLTAGE_PROTECTION_LIST(X)
    #undef X
};

inline const char* to_string(UndervoltageProtection p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case UndervoltageProtection::NAME: return #NAME;
        UNDERVOLTAGE_PROTECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  Undervoltage Protection Levels
//--------------------------------------
/**
 * @brief Enumerates possible levels for undervoltage protection.
 *
 * Used with the SUPPLY_LEVEL parameter to configure the VS undervoltage threshold.
 *
 * Table — Undervoltage Protection Levels:
 *  NUMBER | NAME      | DESCRIPTION
 *  ------ | --------- | -----------------------------------------------------------------
 *     0   | DISABLED  | Comparator disabled.
 *     1   | LEVEL_0   | Hardware level 0.
 *     2   | LEVEL_1   | Hardware level 1.
 *     3   | LEVEL_2   | Hardware level 2.
 *     4   | LEVEL_3   | Hardware level 3.
 *     5   | LEVEL_4   | Hardware level 4.
 *     6   | LEVEL_5   | Hardware level 5.
 *     7   | LEVEL_6   | Hardware level 6.
 *     8   | LEVEL_7   | Hardware level 7.
 *     9   | LEVEL_8   | Hardware level 8.
 *    10   | LEVEL_9   | Hardware level 9.
 *    11   | LEVEL_10  | Hardware level 10.
 *    12   | LEVEL_11  | Hardware level 11.
 *    13   | LEVEL_12  | Hardware level 12.
 *    14   | LEVEL_13  | Hardware level 13.
 *    15   | LEVEL_14  | Hardware level 14.
 *    16   | LEVEL_15  | Hardware level 15.
 */
#define UNDERVOLTAGE_LEVEL_LIST(X) \
    X(DISABLED, 0,  /*!< Comparator disabled. */) \
    X(LEVEL_0,  1,  /*!< Hardware level 0. */) \
    X(LEVEL_1,  2,  /*!< Hardware level 1. */) \
    X(LEVEL_2,  3,  /*!< Hardware level 2. */) \
    X(LEVEL_3,  4,  /*!< Hardware level 3. */) \
    X(LEVEL_4,  5,  /*!< Hardware level 4. */) \
    X(LEVEL_5,  6,  /*!< Hardware level 5. */) \
    X(LEVEL_6,  7,  /*!< Hardware level 6. */) \
    X(LEVEL_7,  8,  /*!< Hardware level 7. */) \
    X(LEVEL_8,  9,  /*!< Hardware level 8. */) \
    X(LEVEL_9,  10, /*!< Hardware level 9. */) \
    X(LEVEL_10, 11, /*!< Hardware level 10. */) \
    X(LEVEL_11, 12, /*!< Hardware level 11. */) \
    X(LEVEL_12, 13, /*!< Hardware level 12. */) \
    X(LEVEL_13, 14, /*!< Hardware level 13. */) \
    X(LEVEL_14, 15, /*!< Hardware level 14. */) \
    X(LEVEL_15, 16, /*!< Hardware level 15. */)

enum class UndervoltageLevel : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    UNDERVOLTAGE_LEVEL_LIST(X)
    #undef X
};

inline const char* to_string(UndervoltageLevel level) {
    switch(level) {
        #define X(NAME, VALUE, DOC) case UndervoltageLevel::NAME: return #NAME;
        UNDERVOLTAGE_LEVEL_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef UNDERVOLTAGE_LEVEL_LIST

//--------------------------------------
//  Undervoltage Protection Enable/Disable
//--------------------------------------
/**
 * @brief Enumerates options for enabling or disabling undervoltage protection.
 *
 * Table — Undervoltage Protection Enable/Disable:
 *  NUMBER | NAME      | DESCRIPTION
 *  ------ | --------- | -----------------------------------------------------------------
 *     0   | DISABLED  | Protection disabled.
 *     1   | ENABLED   | Protection enabled.
 */
#define UNDERVOLTAGE_ENABLE_LIST(X) \
    X(DISABLED, 0, /*!< Protection disabled. */) \
    X(ENABLED,  1, /*!< Protection enabled. */)

enum class UndervoltageEnable : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    UNDERVOLTAGE_ENABLE_LIST(X)
    #undef X
};

inline const char* to_string(UndervoltageEnable enable) {
    switch(enable) {
        #define X(NAME, VALUE, DOC) case UndervoltageEnable::NAME: return #NAME;
        UNDERVOLTAGE_ENABLE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef UNDERVOLTAGE_ENABLE_LIST

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Gate Short (VGS) Protection Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring gate-to-source (VGS) short circuit protection.
 * 
 * These parameters enable/disable protection against gate-to-source voltage shorts for
 * different phases (UVW, Y2) and transitions (ON, OFF) of both low and high side drivers.
 * The protection includes configurable blanking and deglitch times.
 */
#define VGS_SHORT_PROTECTION_LIST(X) \
    X(UVW_LOW_SIDE_ON_ENABLE,  272, /*!< Enable VGS short protection for ON transition of UVW low side [0, 1]. Default: 1. */) \
    X(UVW_LOW_SIDE_OFF_ENABLE, 273, /*!< Enable VGS short protection for OFF transition of UVW low side [0, 1]. Default: 1. */) \
    X(UVW_HIGH_SIDE_ON_ENABLE, 274, /*!< Enable VGS short protection for ON transition of UVW high side [0, 1]. Default: 1. */) \
    X(UVW_HIGH_SIDE_OFF_ENABLE,275, /*!< Enable VGS short protection for OFF transition of UVW high side [0, 1]. Default: 1. */) \
    X(Y2_LOW_SIDE_ON_ENABLE,   276, /*!< Enable VGS short protection for ON transition of Y2 low side [0, 1]. Default: 1. */) \
    X(Y2_LOW_SIDE_OFF_ENABLE,  277, /*!< Enable VGS short protection for OFF transition of Y2 low side [0, 1]. Default: 1. */) \
    X(Y2_HIGH_SIDE_ON_ENABLE,  278, /*!< Enable VGS short protection for ON transition of Y2 high side [0, 1]. Default: 1. */) \
    X(Y2_HIGH_SIDE_OFF_ENABLE, 279, /*!< Enable VGS short protection for OFF transition of Y2 high side [0, 1]. Default: 1. */) \
    X(UVW_BLANKING,            280, /*!< VGS short protection blanking time for UVW phases [0-3]. Default: 1. */) \
    X(Y2_BLANKING,             281, /*!< VGS short protection blanking time for Y2 phase [0-3]. Default: 1. */) \
    X(UVW_DEGLITCH,            282, /*!< VGS short protection deglitch time for UVW phases [0-7]. Default: 1. */) \
    X(Y2_DEGLITCH,             283, /*!< VGS short protection deglitch time for Y2 phase [0-7]. Default: 1. */)

enum class VgsShortProtection : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VGS_SHORT_PROTECTION_LIST(X)
    #undef X
};

inline const char* to_string(VgsShortProtection p) {
    switch(p) {
        #define X(NAME, VALUE, DOC) case VgsShortProtection::NAME: return #NAME;
        VGS_SHORT_PROTECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  VGS Protection Enable/Disable
//--------------------------------------
/**
 * @brief Enumerates options for enabling or disabling VGS short protection.
 *
 * Table — VGS Protection Enable/Disable:
 *  NUMBER | NAME      | DESCRIPTION
 *  ------ | --------- | -----------------------------------------------------------------
 *     0   | DISABLED  | Protection disabled.
 *     1   | ENABLED   | Protection enabled.
 */
#define VGS_SHORT_ENABLE_LIST(X) \
    X(DISABLED, 0, /*!< Protection disabled. */) \
    X(ENABLED,  1, /*!< Protection enabled. */)

enum class VgsShortEnable : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VGS_SHORT_ENABLE_LIST(X)
    #undef X
};

inline const char* to_string(VgsShortEnable e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case VgsShortEnable::NAME: return #NAME;
        VGS_SHORT_ENABLE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VGS_SHORT_ENABLE_LIST

//--------------------------------------
//  VGS Protection Blanking Time
//--------------------------------------
/**
 * @brief Enumerates possible blanking times for VGS short protection.
 *
 * Table — VGS Protection Blanking Time:
 *  NUMBER | NAME             | DESCRIPTION
 *  ------ | ---------------- | -----------------------------------------------------------------
 *     0   | OFF              | No blanking.
 *     1   | T_0_25_MICROSEC  | 0.25 µs blanking time.
 *     2   | T_0_5_MICROSEC   | 0.5 µs blanking time.
 *     3   | T_1_MICROSEC     | 1.0 µs blanking time.
 */
#define VGS_BLANKING_TIME_LIST(X) \
    X(OFF,              0, /*!< No blanking. */) \
    X(T_0_25_MICROSEC,  1, /*!< 0.25 µs blanking time. */) \
    X(T_0_5_MICROSEC,   2, /*!< 0.5 µs blanking time. */) \
    X(T_1_MICROSEC,     3, /*!< 1.0 µs blanking time. */)

enum class VgsBlankingTime : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VGS_BLANKING_TIME_LIST(X)
    #undef X
};

inline const char* to_string(VgsBlankingTime t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case VgsBlankingTime::NAME: return #NAME;
        VGS_BLANKING_TIME_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VGS_BLANKING_TIME_LIST

//--------------------------------------
//  VGS Protection Deglitch Time
//--------------------------------------
/**
 * @brief Enumerates possible deglitch times for VGS short protection.
 *
 * Table — VGS Protection Deglitch Time:
 *  NUMBER | NAME             | DESCRIPTION
 *  ------ | ---------------- | -----------------------------------------------------------------
 *     0   | OFF              | No deglitching.
 *     1   | T_0_25_MICROSEC  | 0.25 µs deglitch time.
 *     2   | T_0_5_MICROSEC   | 0.5 µs deglitch time.
 *     3   | T_1_MICROSEC     | 1.0 µs deglitch time.
 *     4   | T_2_MICROSEC     | 2.0 µs deglitch time.
 *     5   | T_4_MICROSEC     | 4.0 µs deglitch time.
 *     6   | T_6_MICROSEC     | 6.0 µs deglitch time.
 *     7   | T_8_MICROSEC     | 8.0 µs deglitch time.
 */
#define VGS_DEGLITCH_TIME_LIST(X) \
    X(OFF,              0, /*!< No deglitching. */) \
    X(T_0_25_MICROSEC,  1, /*!< 0.25 µs deglitch time. */) \
    X(T_0_5_MICROSEC,   2, /*!< 0.5 µs deglitch time. */) \
    X(T_1_MICROSEC,     3, /*!< 1.0 µs deglitch time. */) \
    X(T_2_MICROSEC,     4, /*!< 2.0 µs deglitch time. */) \
    X(T_4_MICROSEC,     5, /*!< 4.0 µs deglitch time. */) \
    X(T_6_MICROSEC,     6, /*!< 6.0 µs deglitch time. */) \
    X(T_8_MICROSEC,     7, /*!< 8.0 µs deglitch time. */)

enum class VgsDeglitchTime : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VGS_DEGLITCH_TIME_LIST(X)
    #undef X
};

inline const char* to_string(VgsDeglitchTime t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case VgsDeglitchTime::NAME: return #NAME;
        VGS_DEGLITCH_TIME_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VGS_DEGLITCH_TIME_LIST

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███╗   ███╗ ██████╗ ████████╗ ██████╗ ██████╗      ██████╗ ██████╗ ███╗   ██╗███████╗██╗ ██████╗              //
//    ████╗ ████║██╔═══██╗╚══██╔══╝██╔═══██╗██╔══██╗    ██╔════╝██╔═══██╗████╗  ██║██╔════╝██║██╔════╝              //
//    ██╔████╔██║██║   ██║   ██║   ██║   ██║██████╔╝    ██║     ██║   ██║██╔██╗ ██║█████╗  ██║██║  ███╗             //
//    ██║╚██╔╝██║██║   ██║   ██║   ██║   ██║██╔══██╗    ██║     ██║   ██║██║╚██╗██║██╔══╝  ██║██║   ██║             //
//    ██║ ╚═╝ ██║╚██████╔╝   ██║   ╚██████╔╝██║  ██║    ╚██████╗╚██████╔╝██║ ╚████║██║     ██║╚██████╔╝             //
//    ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝  ╚═╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝     ╚═╝ ╚═════╝              //
//                                                                                                                  //
//==================================================================================================================//
//                                               MOTOR CONFIG SECTION                                               //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Motor Configuration Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring motor characteristics and drive settings.
 *
 * Table — Motor Configuration Parameters:
 *  NUMBER | NAME                  | DESCRIPTION
 *  ------ | --------------------- | -----------------------------------------------------------------
 *     0   | MOTOR_TYPE            | Motor type selection. See MotorType enum. Default: 0 (NO_MOTOR). RWE
 *     1   | MOTOR_POLE_PAIRS      | Number of pole pairs for motor [0-127]. Default: 1. RWE
 *     2   | MOTOR_DIRECTION       | Motor direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. RWE
 *     3   | MOTOR_PWM_FREQUENCY   | PWM frequency in Hz [10000-100000]. Default: 25000. RWE
 *     4   | COMMUTATION_MODE      | Motor commutation mode. See CommutationMode enum. Default: 0 (SYSTEM_OFF). RW
 *     5   | OUTPUT_VOLTAGE_LIMIT  | PID UQ/UD output limit for circular limiter [0-32767]. Default: 8000. RWE
 *     8   | PWM_SWITCHING_SCHEME  | PWM switching scheme. See PwmSwitchingScheme enum. Default: 1 (SVPWM). RWE
 */
#define MOTOR_CONFIG_LIST(X) \
    X(MOTOR_TYPE,            0, /*!< Motor type selection. See MotorType enum. Default: 0 (NO_MOTOR). RWE */) \
    X(MOTOR_POLE_PAIRS,      1, /*!< Number of pole pairs for motor [0-127]. Default: 1. RWE */) \
    X(MOTOR_DIRECTION,       2, /*!< Motor direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. RWE */) \
    X(MOTOR_PWM_FREQUENCY,   3, /*!< PWM frequency in Hz [10000-100000]. Default: 25000. RWE */) \
    X(COMMUTATION_MODE,      4, /*!< Motor commutation mode. See CommutationMode enum. Default: 0 (SYSTEM_OFF). RW */) \
    X(OUTPUT_VOLTAGE_LIMIT,  5, /*!< PID UQ/UD output limit for circular limiter [0-32767]. Default: 8000. RWE */) \
    X(PWM_SWITCHING_SCHEME,  8, /*!< PWM switching scheme. See PwmSwitchingScheme enum. Default: 1 (SVPWM). RWE */) \
    X(IDLE_MOTOR_PWM_BEHAVIOR, 9, /*!< Idle motor PWM behavior. */)

enum class MotorConfig : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    MOTOR_CONFIG_LIST(X)
    #undef X
};

inline const char* to_string(MotorConfig config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case MotorConfig::NAME: return #NAME;
        MOTOR_CONFIG_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  Motor Types
//--------------------------------------
/**
 * @brief Enumerates supported motor types.
 *
 * Table — Motor Types:
 *  NUMBER | NAME           | DESCRIPTION
 *  ------ | -------------- | ---------------------------
 *     0   | NO_MOTOR       | No motor selected
 *     1   | DC_MOTOR       | DC motor
 *     2   | STEPPER_MOTOR  | Stepper motor
 *     3   | BLDC_MOTOR     | Brushless DC motor
 */
#define MOTOR_TYPE_LIST(X) \
    X(NO_MOTOR,      0, /*!< No motor selected */) \
    X(DC_MOTOR,      1, /*!< DC motor */) \
    X(STEPPER_MOTOR, 2, /*!< Stepper motor */) \
    X(BLDC_MOTOR,    3, /*!< Brushless DC motor */)

enum class MotorType : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    MOTOR_TYPE_LIST(X)
    #undef X
};

inline const char* to_string(MotorType type) {
    switch(type) {
        #define X(NAME, VALUE, DOC) case MotorType::NAME: return #NAME;
        MOTOR_TYPE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef MOTOR_TYPE_LIST

//--------------------------------------
//  Motor Direction
//--------------------------------------
/**
 * @brief Enumerates motor rotation directions.
 *
 * Table — Motor Direction:
 *  NUMBER | NAME     | DESCRIPTION
 *  ------ | -------- | ---------------------------
 *     0   | FORWARD  | Forward rotation
 *     1   | REVERSE  | Reverse rotation
 */
#define MOTOR_DIRECTION_LIST(X) \
    X(FORWARD, 0, /*!< Forward rotation */) \
    X(REVERSE, 1, /*!< Reverse rotation */)

enum class MotorDirection : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    MOTOR_DIRECTION_LIST(X)
    #undef X
};

inline const char* to_string(MotorDirection direction) {
    switch(direction) {
        #define X(NAME, VALUE, DOC) case MotorDirection::NAME: return #NAME;
        MOTOR_DIRECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef MOTOR_DIRECTION_LIST

//--------------------------------------
//  PWM Switching Schemes
//--------------------------------------
/**
 * @brief Enumerates PWM switching schemes.
 *
 * Table — PWM Switching Schemes:
 *  NUMBER | NAME         | DESCRIPTION
 *  ------ | ------------ | ---------------------------
 *     0   | STANDARD     | Standard modulation
 *     1   | SVPWM        | Space Vector PWM
 *     2   | FLAT_BOTTOM  | Flat bottom modulation
 */
#define PWM_SWITCHING_SCHEME_LIST(X) \
    X(STANDARD,    0, /*!< Standard modulation */) \
    X(SVPWM,       1, /*!< Space Vector PWM */) \
    X(FLAT_BOTTOM, 2, /*!< Flat bottom modulation */)

enum class PwmSwitchingScheme : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    PWM_SWITCHING_SCHEME_LIST(X)
    #undef X
};

inline const char* to_string(PwmSwitchingScheme scheme) {
    switch(scheme) {
        #define X(NAME, VALUE, DOC) case PwmSwitchingScheme::NAME: return #NAME;
        PWM_SWITCHING_SCHEME_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef PWM_SWITCHING_SCHEME_LIST

/// @name Commutation Modes
/// @{
/**
 * @brief Commutation modes define how the motor shaft angle is determined and the system's state.
 *
 * To turn a motor, the commutation mode must be selected using the parameter `COMMUTATION_MODE`.
 * This specifies the feedback system used to determine the motor's shaft angle. It is also used to
 * enable/disable the system or switch to a state with shorted motor coils.
 *
 * ### Commutation Modes and Their Behavior:
 *
 * #### System Off
 * - Default state after power-on and reset events.
 * - Certain operations like changing the motor type require this mode to be active.
 * - PWM behavior is determined by the `IDLE_MOTOR_PWM_BEHAVIOR` parameter:
 *   - `1`: PWM signals are switched off, leaving the motor electrically floating (default).
 *   - `0`: PWM signals remain active.
 * - Note: This mode must be manually selected when exiting from parameter mode to the bootloader.
 *
 * #### System Off, Low-Side FETs On
 * - Charges all low-side motor FETs, shorting the motor coils to ground.
 * - Not used during normal operation.
 *
 * #### System Off, High-Side FETs On
 * - Charges all high-side motor FETs, shorting the motor coils to the supply voltage.
 * - Not used during normal operation.
 *
 * #### FOC (Openloop, Voltage Mode)
 * - Applies a constant duty cycle to the motor.
 * - Voltage is defined by the `OPENLOOP_VOLTAGE` parameter (PWM duty cycle relative to supply voltage).
 * - Intended for initial setup purposes only.
 * - Current is not limited, so configure the `OPENLOOP_VOLTAGE` parameter with care.
 *
 * #### FOC (Openloop, Current Mode)
 * - Applies a constant current to the motor.
 * - Current is defined by the `OPENLOOP_CURRENT` parameter and regulated by the flux control loop.
 * - For stepper/BLDC motors:
 *   - Commutation angle is calculated by the internal hardware ramper block.
 *   - Maximum current is limited by the `MAX_FLUX` parameter.
 *   - Requires setting a velocity target using the `TARGET_VELOCITY` parameter.
 * - For DC motors:
 *   - No velocity input is needed.
 *   - Maximum current is limited by the `MAX_TORQUE` parameter.
 *
 * #### FOC (ABN), FOC (Hall Sensor), FOC (SPI Encoder)
 * - Uses sensor feedback to calculate the motor shaft position.
 * - Feedback source must be configured before activating this mode.
 * - For stepper/BLDC motors:
 *   - Commutation angle is calculated from the selected feedback method.
 *   - Supports torque, velocity, and position control.
 *   - Torque is applied to the motor, and the flux value is assumed zero unless field weakening is active.
 *   - Maximum torque is limited by the `MAX_TORQUE` parameter.
 * - For DC motors:
 *   - Supports velocity or position control.
 *   - Maximum torque is limited by the `MAX_TORQUE` parameter.
 *
 * ### Parameters to Set Up Commutation Mode:
 * - `COMMUTATION_MODE` (Parameter ID: 4)
 *   - 0: SYSTEM_OFF
 *   - 1: SYSTEM_OFF_LOW_SIDE_FETS_ON
 *   - 2: SYSTEM_OFF_HIGH_SIDE_FETS_ON
 *   - 3: FOC_OPENLOOP_VOLTAGE_MODE
 *   - 4: FOC_OPENLOOP_CURRENT_MODE
 *   - 5: FOC_ABN
 *   - 6: FOC_HALL_SENSOR
 *   - 7: RESERVED
 *   - 8: FOC_SPI_ENC
 * - `IDLE_MOTOR_PWM_BEHAVIOR` (Parameter ID: 9)
 *   - 0: PWM_ON_WHEN_MOTOR_IDLE
 *   - 1: PWM_OFF_WHEN_MOTOR_IDLE (default)
 */
/// @}

/// @name Commutation Modes
/// @{
//--------------------------------------
//  Commutation Modes
//--------------------------------------
/**
 * @brief Commutation modes define how the motor shaft angle is determined and the system's state.
 *
 * Table — Commutation Modes:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | -----------------------------------------------------------------
 *     0   | SYSTEM_OFF                  | System off (default after power-on/reset).
 *     1   | SYSTEM_OFF_LOW_SIDE_FETS_ON | All low-side FETs on (coils shorted to ground).
 *     2   | SYSTEM_OFF_HIGH_SIDE_FETS_ON| All high-side FETs on (coils shorted to supply).
 *     3   | FOC_OPENLOOP_VOLTAGE_MODE   | Open-loop voltage mode (constant duty cycle).
 *     4   | FOC_OPENLOOP_CURRENT_MODE   | Open-loop current mode (constant current).
 *     5   | FOC_ABN                     | FOC with ABN encoder feedback.
 *     6   | FOC_HALL_SENSOR             | FOC with Hall sensor feedback.
 *     7   | RESERVED                    | Reserved.
 *     8   | FOC_SPI_ENC                 | FOC with SPI encoder feedback.
 *     9   | IDLE_MOTOR_PWM_BEHAVIOR     | Idle motor PWM behavior.
 */
#define COMMUTATION_MODE_LIST(X) \
    X(SYSTEM_OFF,                  0, /*!< System off (default after power-on/reset). */) \
    X(SYSTEM_OFF_LOW_SIDE_FETS_ON, 1, /*!< All low-side FETs on (coils shorted to ground). */) \
    X(SYSTEM_OFF_HIGH_SIDE_FETS_ON,2, /*!< All high-side FETs on (coils shorted to supply). */) \
    X(FOC_OPENLOOP_VOLTAGE_MODE,   3, /*!< Open-loop voltage mode (constant duty cycle). */) \
    X(FOC_OPENLOOP_CURRENT_MODE,   4, /*!< Open-loop current mode (constant current). */) \
    X(FOC_ABN,                     5, /*!< FOC with ABN encoder feedback. */) \
    X(FOC_HALL_SENSOR,             6, /*!< FOC with Hall sensor feedback. */) \
    X(RESERVED,                    7, /*!< Reserved. */) \
    X(FOC_SPI_ENC,                 8, /*!< FOC with SPI encoder feedback. */) 

enum class CommutationMode : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    COMMUTATION_MODE_LIST(X)
    #undef X
};

inline const char* to_string(CommutationMode mode) {
    switch(mode) {
        #define X(NAME, VALUE, DOC) case CommutationMode::NAME: return #NAME;
        COMMUTATION_MODE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef COMMUTATION_MODE_LIST
/// @}

/// @name Idle Motor PWM Behavior
/// @{
//--------------------------------------
//  Idle Motor PWM Behavior
//--------------------------------------
/**
 * @brief PWM behavior in commutation mode "System Off" (Parameter ID: 9).
 *
 * Table — Idle Motor PWM Behavior:
 *  NUMBER | NAME                     | DESCRIPTION
 *  ------ | ------------------------ | -----------------------------------------------------------------
 *     0   | PWM_ON_WHEN_MOTOR_IDLE   | PWM stays on when motor is idle (all phases same voltage).
 *     1   | PWM_OFF_WHEN_MOTOR_IDLE  | PWM off (high-Z, motor floating) [default].
 */
#define IDLE_MOTOR_PWM_BEHAVIOR_LIST(X) \
    X(PWM_ON_WHEN_MOTOR_IDLE,  0, /*!< PWM stays on when motor is idle (all phases same voltage). */) \
    X(PWM_OFF_WHEN_MOTOR_IDLE, 1, /*!< PWM off (high-Z, motor floating) [default]. */)

enum class IdleMotorPwmBehavior : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    IDLE_MOTOR_PWM_BEHAVIOR_LIST(X)
    #undef X
};

inline const char* to_string(IdleMotorPwmBehavior behavior) {
    switch(behavior) {
        #define X(NAME, VALUE, DOC) case IdleMotorPwmBehavior::NAME: return #NAME;
        IDLE_MOTOR_PWM_BEHAVIOR_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef IDLE_MOTOR_PWM_BEHAVIOR_LIST
/// @}

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  ADC Configuration Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring ADCs for motor current measurement.
 *
 * Table — ADC Configuration Parameters:
 *  NUMBER | NAME                       | DESCRIPTION
 *  ------ | -------------------------- | -----------------------------------------------------------------
 *    12   | ADC_SHUNT_TYPE             | Shunt type for ADC measurements. See AdcShuntType enum. Default: 4 (BOTTOM_SHUNTS).
 *    13   | ADC_I0_RAW                 | Raw ADC measurement for I0 shunt [-32768, 32767]. Read-only.
 *    14   | ADC_I1_RAW                 | Raw ADC measurement for I1 shunt [-32768, 32767]. Read-only.
 *    15   | ADC_I2_RAW                 | Raw ADC measurement for I2 shunt [-32768, 32767]. Read-only.
 *    16   | ADC_I3_RAW                 | Raw ADC measurement for I3 shunt [-32768, 32767]. Read-only.
 *    17   | CSA_GAIN_ADC_I0_TO_ADC_I2  | Current sense amplifier gain for ADC I0, I1 and I2. See CsaGain enum. Default: 1 (GAIN_10X).
 *    18   | CSA_GAIN_ADC_I3            | Current sense amplifier gain for ADC I3. See CsaGain enum. Default: 1 (GAIN_10X).
 *    19   | CSA_FILTER_ADC_I0_TO_ADC_I2| Current sense amplifier filter for ADC I0, I1 and I2. See CsaFilter enum. Default: 0 (T_0_55_MICROSEC).
 *    20   | CSA_FILTER_ADC_I3          | Current sense amplifier filter for ADC I3. See CsaFilter enum. Default: 0 (T_0_55_MICROSEC).
 *    21   | CURRENT_SCALING_FACTOR     | Current scaling factor converting internal units to real-world units [1, 65535]. Default: 520.
 *    22   | PHASE_UX1_ADC_MAPPING      | Mapping ADC to UX1. See AdcMapping enum. Default: 0 (ADC_I0).
 *    23   | PHASE_VX2_ADC_MAPPING      | Mapping ADC to VX2. See AdcMapping enum. Default: 1 (ADC_I1).
 *    24   | PHASE_WY1_ADC_MAPPING      | Mapping ADC to WY1. See AdcMapping enum. Default: 2 (ADC_I2).
 *    25   | PHASE_Y2_ADC_MAPPING       | Mapping ADC to Y2. See AdcMapping enum. Default: 3 (ADC_I3).
 *    26   | ADC_I0_SCALE               | Scaling applied to ADC I0 [1, 32767]. Default: 1024.
 *    27   | ADC_I1_SCALE               | Scaling applied to ADC I1 [1, 32767]. Default: 1024.
 *    28   | ADC_I2_SCALE               | Scaling applied to ADC I2 [1, 32767]. Default: 1024.
 *    29   | ADC_I3_SCALE               | Scaling applied to ADC I3 [1, 32767]. Default: 1024.
 *    30   | ADC_I0_INVERTED            | Invert the reading of ADC I0. 0: NOT_INVERTED, 1: INVERTED. Default: 1.
 *    31   | ADC_I1_INVERTED            | Invert the reading of ADC I1. 0: NOT_INVERTED, 1: INVERTED. Default: 1.
 *    32   | ADC_I2_INVERTED            | Invert the reading of ADC I2. 0: NOT_INVERTED, 1: INVERTED. Default: 1.
 *    33   | ADC_I3_INVERTED            | Invert the reading of ADC I3. 0: NOT_INVERTED, 1: INVERTED. Default: 1.
 *    34   | ADC_I0_OFFSET              | Offset applied to ADC I0 measurement [-32768, 32767]. Default: 0.
 *    35   | ADC_I1_OFFSET              | Offset applied to ADC I1 measurement [-32768, 32767]. Default: 0.
 *    36   | ADC_I2_OFFSET              | Offset applied to ADC I2 measurement [-32768, 32767]. Default: 0.
 *    37   | ADC_I3_OFFSET              | Offset applied to ADC I3 measurement [-32768, 32767]. Default: 0.
 *    38   | ADC_I0                     | Scaled and offset compensated ADC I0 measurement [-32768, 32767]. Read-only.
 *    39   | ADC_I1                     | Scaled and offset compensated ADC I1 measurement [-32768, 32767]. Read-only.
 *    40   | ADC_I2                     | Scaled and offset compensated ADC I2 measurement [-32768, 32767]. Read-only.
 *    41   | ADC_I3                     | Scaled and offset compensated ADC I3 measurement [-32768, 32767]. Read-only.
 */
#define ADC_CONFIG_LIST(X) \
    X(ADC_SHUNT_TYPE,             12, /*!< Shunt type for ADC measurements. See AdcShuntType enum. Default: 4 (BOTTOM_SHUNTS). */) \
    X(ADC_I0_RAW,                 13, /*!< Raw ADC measurement for I0 shunt [-32768, 32767]. Read-only. */) \
    X(ADC_I1_RAW,                 14, /*!< Raw ADC measurement for I1 shunt [-32768, 32767]. Read-only. */) \
    X(ADC_I2_RAW,                 15, /*!< Raw ADC measurement for I2 shunt [-32768, 32767]. Read-only. */) \
    X(ADC_I3_RAW,                 16, /*!< Raw ADC measurement for I3 shunt [-32768, 32767]. Read-only. */) \
    X(CSA_GAIN_ADC_I0_TO_ADC_I2,  17, /*!< Current sense amplifier gain for ADC I0, I1 and I2. See CsaGain enum. Default: 1 (GAIN_10X). */) \
    X(CSA_GAIN_ADC_I3,            18, /*!< Current sense amplifier gain for ADC I3. See CsaGain enum. Default: 1 (GAIN_10X). */) \
    X(CSA_FILTER_ADC_I0_TO_ADC_I2,19, /*!< Current sense amplifier filter for ADC I0, I1 and I2. See CsaFilter enum. Default: 0 (T_0_55_MICROSEC). */) \
    X(CSA_FILTER_ADC_I3,          20, /*!< Current sense amplifier filter for ADC I3. See CsaFilter enum. Default: 0 (T_0_55_MICROSEC). */) \
    X(CURRENT_SCALING_FACTOR,     21, /*!< Current scaling factor converting internal units to real-world units [1, 65535]. Default: 520. */) \
    X(PHASE_UX1_ADC_MAPPING,      22, /*!< Mapping ADC to UX1. See AdcMapping enum. Default: 0 (ADC_I0). */) \
    X(PHASE_VX2_ADC_MAPPING,      23, /*!< Mapping ADC to VX2. See AdcMapping enum. Default: 1 (ADC_I1). */) \
    X(PHASE_WY1_ADC_MAPPING,      24, /*!< Mapping ADC to WY1. See AdcMapping enum. Default: 2 (ADC_I2). */) \
    X(PHASE_Y2_ADC_MAPPING,       25, /*!< Mapping ADC to Y2. See AdcMapping enum. Default: 3 (ADC_I3). */) \
    X(ADC_I0_SCALE,               26, /*!< Scaling applied to ADC I0 [1, 32767]. Default: 1024. */) \
    X(ADC_I1_SCALE,               27, /*!< Scaling applied to ADC I1 [1, 32767]. Default: 1024. */) \
    X(ADC_I2_SCALE,               28, /*!< Scaling applied to ADC I2 [1, 32767]. Default: 1024. */) \
    X(ADC_I3_SCALE,               29, /*!< Scaling applied to ADC I3 [1, 32767]. Default: 1024. */) \
    X(ADC_I0_INVERTED,            30, /*!< Invert the reading of ADC I0. 0: NOT_INVERTED, 1: INVERTED. Default: 1. */) \
    X(ADC_I1_INVERTED,            31, /*!< Invert the reading of ADC I1. 0: NOT_INVERTED, 1: INVERTED. Default: 1. */) \
    X(ADC_I2_INVERTED,            32, /*!< Invert the reading of ADC I2. 0: NOT_INVERTED, 1: INVERTED. Default: 1. */) \
    X(ADC_I3_INVERTED,            33, /*!< Invert the reading of ADC I3. 0: NOT_INVERTED, 1: INVERTED. Default: 1. */) \
    X(ADC_I0_OFFSET,              34, /*!< Offset applied to ADC I0 measurement [-32768, 32767]. Default: 0. */) \
    X(ADC_I1_OFFSET,              35, /*!< Offset applied to ADC I1 measurement [-32768, 32767]. Default: 0. */) \
    X(ADC_I2_OFFSET,              36, /*!< Offset applied to ADC I2 measurement [-32768, 32767]. Default: 0. */) \
    X(ADC_I3_OFFSET,              37, /*!< Offset applied to ADC I3 measurement [-32768, 32767]. Default: 0. */) \
    X(ADC_I0,                     38, /*!< Scaled and offset compensated ADC I0 measurement [-32768, 32767]. Read-only. */) \
    X(ADC_I1,                     39, /*!< Scaled and offset compensated ADC I1 measurement [-32768, 32767]. Read-only. */) \
    X(ADC_I2,                     40, /*!< Scaled and offset compensated ADC I2 measurement [-32768, 32767]. Read-only. */) \
    X(ADC_I3,                     41, /*!< Scaled and offset compensated ADC I3 measurement [-32768, 32767]. Read-only. */)

enum class AdcConfig : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ADC_CONFIG_LIST(X)
    #undef X
};

inline const char* to_string(AdcConfig config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case AdcConfig::NAME: return #NAME;
        ADC_CONFIG_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  ADC Shunt Types
//--------------------------------------
/**
 * @brief Enumerates shunt types for ADC measurements.
 *
 * Table — ADC Shunt Types:
 *  NUMBER | NAME          | DESCRIPTION
 *  ------ | ------------- | -----------------------------------------------
 *     0   | INLINE_UVW    | Inline shunts for U, V, W phases
 *     1   | INLINE_VW     | Inline shunts for V, W phases
 *     2   | INLINE_UW     | Inline shunts for U, W phases
 *     3   | INLINE_UV     | Inline shunts for U, V phases
 *     4   | BOTTOM_SHUNTS | Bottom shunts
 */
#define ADC_SHUNT_TYPE_LIST(X) \
    X(INLINE_UVW,    0, /*!< Inline shunts for U, V, W phases */) \
    X(INLINE_VW,     1, /*!< Inline shunts for V, W phases */) \
    X(INLINE_UW,     2, /*!< Inline shunts for U, W phases */) \
    X(INLINE_UV,     3, /*!< Inline shunts for U, V phases */) \
    X(BOTTOM_SHUNTS, 4, /*!< Bottom shunts */)

enum class AdcShuntType : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ADC_SHUNT_TYPE_LIST(X)
    #undef X
};

inline const char* to_string(AdcShuntType t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case AdcShuntType::NAME: return #NAME;
        ADC_SHUNT_TYPE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ADC_SHUNT_TYPE_LIST

//--------------------------------------
//  Current Sense Amplifier Gain
//--------------------------------------
/**
 * @brief Enumerates current sense amplifier gain settings.
 *
 * Table — Current Sense Amplifier Gain:
 *  NUMBER | NAME              | DESCRIPTION
 *  ------ | ----------------- | -----------------------------------------------
 *     0   | GAIN_5X           | 5x gain
 *     1   | GAIN_10X          | 10x gain
 *     2   | GAIN_20X          | 20x gain
 *     3   | GAIN_40X          | 40x gain
 *     4   | GAIN_1X_BYPASS_CSA| 1x gain (bypass CSA)
 */
#define CSA_GAIN_LIST(X) \
    X(GAIN_5X,           0, /*!< 5x gain */) \
    X(GAIN_10X,          1, /*!< 10x gain */) \
    X(GAIN_20X,          2, /*!< 20x gain */) \
    X(GAIN_40X,          3, /*!< 40x gain */) \
    X(GAIN_1X_BYPASS_CSA, 4, /*!< 1x gain (bypass CSA) */)

enum class CsaGain : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    CSA_GAIN_LIST(X)
    #undef X
};

inline const char* to_string(CsaGain g) {
    switch(g) {
        #define X(NAME, VALUE, DOC) case CsaGain::NAME: return #NAME;
        CSA_GAIN_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef CSA_GAIN_LIST

//--------------------------------------
//  Current Sense Amplifier Filter
//--------------------------------------
/**
 * @brief Enumerates current sense amplifier filter settings.
 *
 * Table — Current Sense Amplifier Filter:
 *  NUMBER | NAME              | DESCRIPTION
 *  ------ | ----------------- | -----------------------------------------------
 *     0   | T_0_55_MICROSEC   | 0.55 μs filter time
 *     1   | T_0_75_MICROSEC   | 0.75 μs filter time
 *     2   | T_1_0_MICROSEC    | 1.0 μs filter time
 *     3   | T_1_35_MICROSEC   | 1.35 μs filter time
 */
#define CSA_FILTER_LIST(X) \
    X(T_0_55_MICROSEC, 0, /*!< 0.55 μs filter time */) \
    X(T_0_75_MICROSEC, 1, /*!< 0.75 μs filter time */) \
    X(T_1_0_MICROSEC,  2, /*!< 1.0 μs filter time */) \
    X(T_1_35_MICROSEC, 3, /*!< 1.35 μs filter time */)

enum class CsaFilter : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    CSA_FILTER_LIST(X)
    #undef X
};

inline const char* to_string(CsaFilter f) {
    switch(f) {
        #define X(NAME, VALUE, DOC) case CsaFilter::NAME: return #NAME;
        CSA_FILTER_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef CSA_FILTER_LIST

//--------------------------------------
//  ADC Mapping
//--------------------------------------
/**
 * @brief Enumerates ADC mapping options for motor phases.
 *
 * Table — ADC Mapping:
 *  NUMBER | NAME    | DESCRIPTION
 *  ------ | ------- | -----------------------------------------------
 *     0   | ADC_I0  | Map to ADC I0
 *     1   | ADC_I1  | Map to ADC I1
 *     2   | ADC_I2  | Map to ADC I2
 *     3   | ADC_I3  | Map to ADC I3
 */
#define ADC_MAPPING_LIST(X) \
    X(ADC_I0, 0, /*!< Map to ADC I0 */) \
    X(ADC_I1, 1, /*!< Map to ADC I1 */) \
    X(ADC_I2, 2, /*!< Map to ADC I2 */) \
    X(ADC_I3, 3, /*!< Map to ADC I3 */)

enum class AdcMapping : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ADC_MAPPING_LIST(X)
    #undef X
};

inline const char* to_string(AdcMapping m) {
    switch(m) {
        #define X(NAME, VALUE, DOC) case AdcMapping::NAME: return #NAME;
        ADC_MAPPING_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ADC_MAPPING_LIST

//--------------------------------------
//  ADC Inversion Settings
//--------------------------------------
/**
 * @brief Enumerates ADC inversion settings.
 *
 * Table — ADC Inversion Settings:
 *  NUMBER | NAME          | DESCRIPTION
 *  ------ | ------------- | -----------------------------------------------
 *     0   | NOT_INVERTED  | Normal reading (not inverted)
 *     1   | INVERTED      | Inverted reading
 */
#define ADC_INVERSION_LIST(X) \
    X(NOT_INVERTED, 0, /*!< Normal reading (not inverted) */) \
    X(INVERTED,     1, /*!< Inverted reading */)

enum class AdcInversion : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ADC_INVERSION_LIST(X)
    #undef X
};

inline const char* to_string(AdcInversion i) {
    switch(i) {
        #define X(NAME, VALUE, DOC) case AdcInversion::NAME: return #NAME;
        ADC_INVERSION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ADC_INVERSION_LIST

//--------------------------------------
//  PWM Frequency Configuration
//--------------------------------------
/**
 * @brief Enumerates recommended PWM frequencies for different motor types.
 *
 * Table — PWM Frequency Configuration:
 *  NUMBER | NAME               | DESCRIPTION
 *  ------ | ------------------ | -----------------------------------------------------------
 *  25000  | STANDARD_BLDC      | Standard frequency for typical BLDC motors
 *  20000  | STANDARD_STEPPER   | Standard frequency for high-inductance stepper motors
 *  50000  | FAST_BLDC          | For fast-spinning BLDC motors (>10,000 RPM)
 * 100000  | ULTRA_FAST_BLDC    | For very high-speed or low-inductance BLDC motors
 *  20000  | MINIMUM_SILENT     | Minimum frequency to avoid audible switching noise
 */
#define PWM_FREQUENCY_LIST(X) \
    X(STANDARD_BLDC,      25000,  /*!< Standard frequency for typical BLDC motors */) \
    X(STANDARD_STEPPER,   20000,  /*!< Standard frequency for high-inductance stepper motors */) \
    X(FAST_BLDC,          50000,  /*!< For fast-spinning BLDC motors (>10,000 RPM) */) \
    X(ULTRA_FAST_BLDC,   100000,  /*!< For very high-speed or low-inductance BLDC motors */) \
    X(MINIMUM_SILENT,     20001,  /*!< Minimum frequency to avoid audible switching noise */)

enum class PwmFrequency : uint32_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    PWM_FREQUENCY_LIST(X)
    #undef X
};

inline const char* to_string(PwmFrequency f) {
    switch(f) {
        #define X(NAME, VALUE, DOC) case PwmFrequency::NAME: return #NAME;
        PWM_FREQUENCY_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef PWM_FREQUENCY_LIST

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗███████╗███████╗██████╗ ██████╗  █████╗  ██████╗██╗  ██╗                                              //
//    ██╔════╝██╔════╝██╔════╝██╔══██╗██╔══██╗██╔══██╗██╔════╝██║ ██╔╝                                              //
//    █████╗  █████╗  █████╗  ██║  ██║██████╔╝███████║██║     █████╔╝                                               //
//    ██╔══╝  ██╔══╝  ██╔══╝  ██║  ██║██╔══██╗██╔══██║██║     ██╔═██╗                                               //
//    ██║     ███████╗███████╗██████╔╝██████╔╝██║  ██║╚██████╗██║  ██╗                                              //
//    ╚═╝     ╚══════╝╚══════╝╚═════╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝                                              //
//                                                                                                                  //
//     ███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗ ███████╗                                                  //
//     ██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗██╔════╝                                                  //
//     ███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝███████╗                                                  //
//     ╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗╚════██║                                                  //
//     ███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║███████║                                                  //
//     ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝                                                  //
//                                                                                                                  //
//==================================================================================================================//
//                                              FEEDBACK SENSOR SECTION                                             //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

//--------------------------------------
//  Feedback Sensor Configuration Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring feedback sensors (ABN, Hall, SPI encoders).
 *
 * Table — Feedback Sensor Configuration Parameters:
 *  NUMBER | NAME                          | DESCRIPTION
 *  ------ | ----------------------------- | -----------------------------------------------------------------
 *     89  | ABN_1_PHI_E                   | Phi_e calculated from ABN feedback [-32768, 32767]. Read-only.
 *     90  | ABN_1_STEPS                   | ABN encoder steps per rotation (CPR) [0, 16777215]. Default: 65536.
 *     91  | ABN_1_DIRECTION               | ABN encoder rotation direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0.
 *     92  | ABN_1_INIT_METHOD             | ABN initialization method. See AbnInitMethod enum. Default: 0.
 *     93  | ABN_1_INIT_STATE              | ABN initialization state. See AbnInitState enum. Read-only.
 *     94  | ABN_1_INIT_DELAY              | Delay for forced phi_e initialization [1000, 10000 ms]. Default: 1000.
 *     95  | ABN_1_INIT_VELOCITY           | Init velocity for N-channel offset [-200000, 200000]. Default: 5.
 *     96  | ABN_1_N_CHANNEL_PHI_E_OFFSET  | Offset between phi_e zero and ABN index pulse [-32768, 32767]. Default: 0.
 *     97  | ABN_1_N_CHANNEL_INVERTED      | ABN N-channel inversion. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. Default: 0.
 *     98  | ABN_1_N_CHANNEL_FILTERING     | ABN N-channel filtering. See AbnNChannelFiltering enum. Default: 0.
 *     99  | ABN_1_CLEAR_ON_NEXT_NULL      | Clear position on next N-channel event. 0: DISABLED, 1: ENABLED. Default: 0.
 *    100  | ABN_1_VALUE                   | Raw ABN encoder counter value [0, 16777215]. Read-only.
 *     74  | HALL_PHI_E                    | Phi_e calculated from Hall feedback [-32768, 32767]. Read-only.
 *     75  | HALL_SECTOR_OFFSET            | Hall sector offset. See HallSectorOffset enum. Default: 0.
 *     76  | HALL_FILTER_LENGTH            | Hall signal filter length [0, 255]. Default: 0.
 *     77  | HALL_POSITION_0_OFFSET        | Hall offset for 0-degree position [-32768, 32767]. Default: 0.
 *     78  | HALL_POSITION_60_OFFSET       | Hall offset for 60-degree position [-32768, 32767]. Default: 10922.
 *     79  | HALL_POSITION_120_OFFSET      | Hall offset for 120-degree position [-32768, 32767]. Default: 21845.
 *     80  | HALL_POSITION_180_OFFSET      | Hall offset for 180-degree position [-32768, 32767]. Default: -32768.
 *     81  | HALL_POSITION_240_OFFSET      | Hall offset for 240-degree position [-32768, 32767]. Default: -21846.
 *     82  | HALL_POSITION_300_OFFSET      | Hall offset for 300-degree position [-32768, 32767]. Default: -10923.
 *     83  | HALL_INVERT_DIRECTION         | Invert Hall angle direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0.
 *     84  | HALL_EXTRAPOLATION_ENABLE     | Enable Hall extrapolation. 0: DISABLED, 1: ENABLED. Default: 0.
 *     85  | HALL_PHI_E_OFFSET             | Hall sensor mounting tolerance compensation [-32768, 32767]. Default: 0.
 *    181  | SPI_ENCODER_CS_SETTLE_DELAY_TIME | CS settle delay time [0, 6375 ns]. Default: 0.
 *    182  | SPI_ENCODER_CS_IDLE_DELAY_TIME   | CS idle delay time between frames [0, 102 us]. Default: 0.
 *    183  | SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE | Size of first SPI transfer frame [1, 16]. Default: 1.
 *    184  | SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE | Size of optional secondary SPI transfer [0, 15]. Default: 0.
 *    185  | SPI_ENCODER_TRANSFER_DATA_3_0   | Transmit data and read received data (bytes 0-3). Default: 0.
 *    186  | SPI_ENCODER_TRANSFER_DATA_7_4   | Transmit data and read received data (bytes 4-7). Default: 0.
 *    187  | SPI_ENCODER_TRANSFER_DATA_11_8  | Transmit data and read received data (bytes 8-11). Default: 0.
 *    188  | SPI_ENCODER_TRANSFER_DATA_15_12 | Transmit data and read received data (bytes 12-15). Default: 0.
 *    189  | SPI_ENCODER_TRANSFER            | SPI interface control. See SpiEncoderTransfer enum. Default: 0.
 *    190  | SPI_ENCODER_POSITION_COUNTER_MASK | Mask to extract position from received data [0, 4294967295]. Default: 0.
 *    191  | SPI_ENCODER_POSITION_COUNTER_SHIFT | Right bit shift for position value before mask [0, 127]. Default: 0.
 *    192  | SPI_ENCODER_POSITION_COUNTER_VALUE | Actual SPI encoder position value [0, 4294967295]. Read-only.
 *    193  | SPI_ENCODER_COMMUTATION_ANGLE   | Actual absolute encoder angle value [-32768, 32767]. Read-only.
 *    194  | SPI_ENCODER_INITIALIZATION_METHOD | Encoder initialization method. See SpiInitMethod enum. Default: 0.
 *    195  | SPI_ENCODER_DIRECTION           | SPI encoder direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0.
 *    196  | SPI_ENCODER_OFFSET              | Internal commutation offset [0, 4294967295]. Default: 0.
 *    197  | SPI_LUT_CORRECTION_ENABLE       | Enable lookup table correction. 0: DISABLED, 1: ENABLED. Default: 0.
 *    198  | SPI_LUT_ADDRESS_SELECT          | Address to read/write in lookup table [0, 255]. Default: 0.
 *    199  | SPI_LUT_DATA                    | Data to read/write to lookup table address [-128, 127]. Default: 0.
 *    201  | SPI_LUT_COMMON_SHIFT_FACTOR     | LUT entries are multiplied with 2^SHIFT_FACTOR [-128, 127]. Default: 0.
 *    174  | ABN_2_STEPS                     | ABN 2 encoder steps per rotation (CPR) [0, 16777215]. Default: 1024.
 *    175  | ABN_2_DIRECTION                 | ABN 2 encoder rotation direction. 0: NORMAL, 1: INVERTED. Default: 0.
 *    176  | ABN_2_GEAR_RATIO                | ABN 2 encoder gear ratio [1, 255]. Default: 1.
 *    177  | ABN_2_ENABLE                    | Enable ABN 2 encoder. 0: DISABLED, 1: ENABLED. Default: 0.
 *    178  | ABN_2_VALUE                     | Raw ABN 2 encoder counter value [0, 4294967295]. Read-only.
 */
#define FEEDBACK_SENSOR_CONFIG_LIST(X) \
    X(ABN_1_PHI_E,                   89,  /*!< Phi_e calculated from ABN feedback [-32768, 32767]. Read-only. */) \
    X(ABN_1_STEPS,                   90,  /*!< ABN encoder steps per rotation (CPR) [0, 16777215]. Default: 65536. */) \
    X(ABN_1_DIRECTION,               91,  /*!< ABN encoder rotation direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. */) \
    X(ABN_1_INIT_METHOD,             92,  /*!< ABN initialization method. See AbnInitMethod enum. Default: 0. */) \
    X(ABN_1_INIT_STATE,              93,  /*!< ABN initialization state. See AbnInitState enum. Read-only. */) \
    X(ABN_1_INIT_DELAY,              94,  /*!< Delay for forced phi_e initialization [1000, 10000 ms]. Default: 1000. */) \
    X(ABN_1_INIT_VELOCITY,           95,  /*!< Init velocity for N-channel offset [-200000, 200000]. Default: 5. */) \
    X(ABN_1_N_CHANNEL_PHI_E_OFFSET,  96,  /*!< Offset between phi_e zero and ABN index pulse [-32768, 32767]. Default: 0. */) \
    X(ABN_1_N_CHANNEL_INVERTED,      97,  /*!< ABN N-channel inversion. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. Default: 0. */) \
    X(ABN_1_N_CHANNEL_FILTERING,     98,  /*!< ABN N-channel filtering. See AbnNChannelFiltering enum. Default: 0. */) \
    X(ABN_1_CLEAR_ON_NEXT_NULL,      99,  /*!< Clear position on next N-channel event. 0: DISABLED, 1: ENABLED. Default: 0. */) \
    X(ABN_1_VALUE,                  100,  /*!< Raw ABN encoder counter value [0, 16777215]. Read-only. */) \
    X(HALL_PHI_E,                    74,  /*!< Phi_e calculated from Hall feedback [-32768, 32767]. Read-only. */) \
    X(HALL_SECTOR_OFFSET,            75,  /*!< Hall sector offset. See HallSectorOffset enum. Default: 0. */) \
    X(HALL_FILTER_LENGTH,            76,  /*!< Hall signal filter length [0, 255]. Default: 0. */) \
    X(HALL_POSITION_0_OFFSET,        77,  /*!< Hall offset for 0-degree position [-32768, 32767]. Default: 0. */) \
    X(HALL_POSITION_60_OFFSET,       78,  /*!< Hall offset for 60-degree position [-32768, 32767]. Default: 10922. */) \
    X(HALL_POSITION_120_OFFSET,      79,  /*!< Hall offset for 120-degree position [-32768, 32767]. Default: 21845. */) \
    X(HALL_POSITION_180_OFFSET,      80,  /*!< Hall offset for 180-degree position [-32768, 32767]. Default: -32768. */) \
    X(HALL_POSITION_240_OFFSET,      81,  /*!< Hall offset for 240-degree position [-32768, 32767]. Default: -21846. */) \
    X(HALL_POSITION_300_OFFSET,      82,  /*!< Hall offset for 300-degree position [-32768, 32767]. Default: -10923. */) \
    X(HALL_INVERT_DIRECTION,         83,  /*!< Invert Hall angle direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. */) \
    X(HALL_EXTRAPOLATION_ENABLE,     84,  /*!< Enable Hall extrapolation. 0: DISABLED, 1: ENABLED. Default: 0. */) \
    X(HALL_PHI_E_OFFSET,             85,  /*!< Hall sensor mounting tolerance compensation [-32768, 32767]. Default: 0. */) \
    X(SPI_ENCODER_CS_SETTLE_DELAY_TIME,    181, /*!< CS settle delay time [0, 6375 ns]. Default: 0. */) \
    X(SPI_ENCODER_CS_IDLE_DELAY_TIME,      182, /*!< CS idle delay time between frames [0, 102 us]. Default: 0. */) \
    X(SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE,  183, /*!< Size of first SPI transfer frame [1, 16]. Default: 1. */) \
    X(SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE, 184, /*!< Size of optional secondary SPI transfer [0, 15]. Default: 0. */) \
    X(SPI_ENCODER_TRANSFER_DATA_3_0,       185, /*!< Transmit data and read received data (bytes 0-3). Default: 0. */) \
    X(SPI_ENCODER_TRANSFER_DATA_7_4,       186, /*!< Transmit data and read received data (bytes 4-7). Default: 0. */) \
    X(SPI_ENCODER_TRANSFER_DATA_11_8,      187, /*!< Transmit data and read received data (bytes 8-11). Default: 0. */) \
    X(SPI_ENCODER_TRANSFER_DATA_15_12,     188, /*!< Transmit data and read received data (bytes 12-15). Default: 0. */) \
    X(SPI_ENCODER_TRANSFER,                189, /*!< SPI interface control. See SpiEncoderTransfer enum. Default: 0. */) \
    X(SPI_ENCODER_POSITION_COUNTER_MASK,   190, /*!< Mask to extract position from received data [0, 4294967295]. Default: 0. */) \
    X(SPI_ENCODER_POSITION_COUNTER_SHIFT,  191, /*!< Right bit shift for position value before mask [0, 127]. Default: 0. */) \
    X(SPI_ENCODER_POSITION_COUNTER_VALUE,  192, /*!< Actual SPI encoder position value [0, 4294967295]. Read-only. */) \
    X(SPI_ENCODER_COMMUTATION_ANGLE,       193, /*!< Actual absolute encoder angle value [-32768, 32767]. Read-only. */) \
    X(SPI_ENCODER_INITIALIZATION_METHOD,   194, /*!< Encoder initialization method. See SpiInitMethod enum. Default: 0. */) \
    X(SPI_ENCODER_DIRECTION,               195, /*!< SPI encoder direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. */) \
    X(SPI_ENCODER_OFFSET,                  196, /*!< Internal commutation offset [0, 4294967295]. Default: 0. */) \
    X(SPI_LUT_CORRECTION_ENABLE,           197, /*!< Enable lookup table correction. 0: DISABLED, 1: ENABLED. Default: 0. */) \
    X(SPI_LUT_ADDRESS_SELECT,              198, /*!< Address to read/write in lookup table [0, 255]. Default: 0. */) \
    X(SPI_LUT_DATA,                        199, /*!< Data to read/write to lookup table address [-128, 127]. Default: 0. */) \
    X(SPI_LUT_COMMON_SHIFT_FACTOR,         201, /*!< LUT entries are multiplied with 2^SHIFT_FACTOR [-128, 127]. Default: 0. */) \
    X(ABN_2_STEPS,                         174, /*!< ABN 2 encoder steps per rotation (CPR) [0, 16777215]. Default: 1024. */) \
    X(ABN_2_DIRECTION,                     175, /*!< ABN 2 encoder rotation direction. 0: NORMAL, 1: INVERTED. Default: 0. */) \
    X(ABN_2_GEAR_RATIO,                    176, /*!< ABN 2 encoder gear ratio [1, 255]. Default: 1. */) \
    X(ABN_2_ENABLE,                        177, /*!< Enable ABN 2 encoder. 0: DISABLED, 1: ENABLED. Default: 0. */) \
    X(ABN_2_VALUE,                         178, /*!< Raw ABN 2 encoder counter value [0, 4294967295]. Read-only. */)

enum class FeedbackSensorConfig : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    FEEDBACK_SENSOR_CONFIG_LIST(X)
    #undef X
};

inline const char* to_string(FeedbackSensorConfig config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case FeedbackSensorConfig::NAME: return #NAME;
        FEEDBACK_SENSOR_CONFIG_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

//--------------------------------------
//  ABN Initialization Methods
//--------------------------------------
/**
 * @brief Enumerates ABN encoder initialization methods.
 *
 * Table — ABN Initialization Methods:
 *  NUMBER | NAME                              | DESCRIPTION
 *  ------ | --------------------------------- | -----------------------------------------------
 *     0   | FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING | Force phi_e zero with active swing.
 *     1   | FORCED_PHI_E_90_ZERO             | Force phi_e 90-degree then zero.
 *     2   | USE_HALL                         | Use Hall sensor for alignment.
 *     3   | USE_N_CHANNEL_OFFSET             | Use N-channel offset for alignment.
 */
#define ABN_INIT_METHOD_LIST(X) \
    X(FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING, 0, /*!< Force phi_e zero with active swing. */) \
    X(FORCED_PHI_E_90_ZERO,               1, /*!< Force phi_e 90-degree then zero. */) \
    X(USE_HALL,                           2, /*!< Use Hall sensor for alignment. */) \
    X(USE_N_CHANNEL_OFFSET,               3, /*!< Use N-channel offset for alignment. */)

enum class AbnInitMethod : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ABN_INIT_METHOD_LIST(X)
    #undef X
};

inline const char* to_string(AbnInitMethod method) {
    switch(method) {
        #define X(NAME, VALUE, DOC) case AbnInitMethod::NAME: return #NAME;
        ABN_INIT_METHOD_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ABN_INIT_METHOD_LIST

//--------------------------------------
//  ABN Initialization States
//--------------------------------------
/**
 * @brief Enumerates ABN encoder initialization states.
 *
 * Table — ABN Initialization States:
 *  NUMBER | NAME  | DESCRIPTION
 *  ------ | ----- | -----------------------------------------------
 *     0   | IDLE  | Initialization idle.
 *     1   | BUSY  | Initialization in progress.
 *     2   | WAIT  | Waiting for completion.
 *     3   | DONE  | Initialization completed.
 */
#define ABN_INIT_STATE_LIST(X) \
    X(IDLE, 0, /*!< Initialization idle. */) \
    X(BUSY, 1, /*!< Initialization in progress. */) \
    X(WAIT, 2, /*!< Waiting for completion. */) \
    X(DONE, 3, /*!< Initialization completed. */)

enum class AbnInitState : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ABN_INIT_STATE_LIST(X)
    #undef X
};

inline const char* to_string(AbnInitState state) {
    switch(state) {
        #define X(NAME, VALUE, DOC) case AbnInitState::NAME: return #NAME;
        ABN_INIT_STATE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ABN_INIT_STATE_LIST

//--------------------------------------
//  ABN N-Channel Filtering Modes
//--------------------------------------
/**
 * @brief Enumerates ABN N-channel filtering modes.
 *
 * Table — ABN N-Channel Filtering Modes:
 *  NUMBER | NAME                     | DESCRIPTION
 *  ------ | ------------------------ | -----------------------------------------------
 *     0   | FILTERING_OFF            | No filtering.
 *     1   | N_EVENT_ON_A_HIGH_B_HIGH | Event on A high, B high.
 *     2   | N_EVENT_ON_A_HIGH_B_LOW  | Event on A high, B low.
 *     3   | N_EVENT_ON_A_LOW_B_HIGH  | Event on A low, B high.
 *     4   | N_EVENT_ON_A_LOW_B_LOW   | Event on A low, B low.
 */
#define ABN_N_CHANNEL_FILTERING_LIST(X) \
    X(FILTERING_OFF,            0, /*!< No filtering. */) \
    X(N_EVENT_ON_A_HIGH_B_HIGH, 1, /*!< Event on A high, B high. */) \
    X(N_EVENT_ON_A_HIGH_B_LOW,  2, /*!< Event on A high, B low. */) \
    X(N_EVENT_ON_A_LOW_B_HIGH,  3, /*!< Event on A low, B high. */) \
    X(N_EVENT_ON_A_LOW_B_LOW,   4, /*!< Event on A low, B low. */)

enum class AbnNChannelFiltering : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ABN_N_CHANNEL_FILTERING_LIST(X)
    #undef X
};

inline const char* to_string(AbnNChannelFiltering filtering) {
    switch(filtering) {
        #define X(NAME, VALUE, DOC) case AbnNChannelFiltering::NAME: return #NAME;
        ABN_N_CHANNEL_FILTERING_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ABN_N_CHANNEL_FILTERING_LIST

//--------------------------------------
//  Hall Sector Offsets
//--------------------------------------
/**
 * @brief Enumerates Hall sector offsets.
 *
 * Table — Hall Sector Offsets:
 *  NUMBER | NAME    | DESCRIPTION
 *  ------ | ------- | -----------------------------------------------
 *     0   | DEG_0   | 0 degrees.
 *     1   | DEG_60  | 60 degrees.
 *     2   | DEG_120 | 120 degrees.
 *     3   | DEG_180 | 180 degrees.
 *     4   | DEG_240 | 240 degrees.
 *     5   | DEG_300 | 300 degrees.
 */
#define HALL_SECTOR_OFFSET_LIST(X) \
    X(DEG_0,   0, /*!< 0 degrees. */) \
    X(DEG_60,  1, /*!< 60 degrees. */) \
    X(DEG_120, 2, /*!< 120 degrees. */) \
    X(DEG_180, 3, /*!< 180 degrees. */) \
    X(DEG_240, 4, /*!< 240 degrees. */) \
    X(DEG_300, 5, /*!< 300 degrees. */)

enum class HallSectorOffset : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    HALL_SECTOR_OFFSET_LIST(X)
    #undef X
};

inline const char* to_string(HallSectorOffset offset) {
    switch(offset) {
        #define X(NAME, VALUE, DOC) case HallSectorOffset::NAME: return #NAME;
        HALL_SECTOR_OFFSET_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef HALL_SECTOR_OFFSET_LIST

//--------------------------------------
//  SPI Encoder Transfer Modes
//--------------------------------------
/**
 * @brief Enumerates SPI encoder transfer modes.
 *
 * Table — SPI Encoder Transfer Modes:
 *  NUMBER | NAME                              | DESCRIPTION
 *  ------ | --------------------------------- | -----------------------------------------------
 *     0   | OFF                               | SPI encoder interface off.
 *     1   | TRIGGER_SINGLE_TRANSFER           | Trigger a single SPI transfer.
 *     2   | CONTINUOUS_POSITION_COUNTER_READ  | Continuously read position counter.
 */
#define SPI_ENCODER_TRANSFER_LIST(X) \
    X(OFF,                             0, /*!< SPI encoder interface off. */) \
    X(TRIGGER_SINGLE_TRANSFER,         1, /*!< Trigger a single SPI transfer. */) \
    X(CONTINUOUS_POSITION_COUNTER_READ, 2, /*!< Continuously read position counter. */)

enum class SpiEncoderTransfer : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    SPI_ENCODER_TRANSFER_LIST(X)
    #undef X
};

inline const char* to_string(SpiEncoderTransfer transfer) {
    switch(transfer) {
        #define X(NAME, VALUE, DOC) case SpiEncoderTransfer::NAME: return #NAME;
        SPI_ENCODER_TRANSFER_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef SPI_ENCODER_TRANSFER_LIST

//--------------------------------------
//  SPI Encoder Initialization Methods
//--------------------------------------
/**
 * @brief Enumerates SPI encoder initialization methods.
 *
 * Table — SPI Encoder Initialization Methods:
 *  NUMBER | NAME                              | DESCRIPTION
 *  ------ | --------------------------------- | -----------------------------------------------
 *     0   | FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING | Force rotor into PHI_E zero with active swing.
 *     1   | FORCED_PHI_E_90_ZERO             | Force rotor into PHI_E 90° then 0° position.
 *     2   | USE_OFFSET                       | Use the offset value stored in SPI_ENCODER_OFFSET.
 */
#define SPI_INIT_METHOD_LIST(X) \
    X(FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING, 0, /*!< Force rotor into PHI_E zero with active swing. */) \
    X(FORCED_PHI_E_90_ZERO,                1, /*!< Force rotor into PHI_E 90° then 0° position. */) \
    X(USE_OFFSET,                          2, /*!< Use the offset value stored in SPI_ENCODER_OFFSET. */)

enum class SpiInitMethod : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    SPI_INIT_METHOD_LIST(X)
    #undef X
};

inline const char* to_string(SpiInitMethod method) {
    switch(method) {
        #define X(NAME, VALUE, DOC) case SpiInitMethod::NAME: return #NAME;
        SPI_INIT_METHOD_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef SPI_INIT_METHOD_LIST

//--------------------------------------
//  Enable/Disable Settings
//--------------------------------------
/**
 * @brief Generic enable/disable settings used by various parameters.
 *
 * Table — Enable/Disable Settings:
 *  NUMBER | NAME      | DESCRIPTION
 *  ------ | --------- | -----------------------------------------------
 *     0   | DISABLED  | Feature disabled.
 *     1   | ENABLED   | Feature enabled.
 */
#define ENABLE_DISABLE_LIST(X) \
    X(DISABLED, 0, /*!< Feature disabled. */) \
    X(ENABLED,  1, /*!< Feature enabled. */)

enum class EnableDisable : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ENABLE_DISABLE_LIST(X)
    #undef X
};

inline const char* to_string(EnableDisable setting) {
    switch(setting) {
        #define X(NAME, VALUE, DOC) case EnableDisable::NAME: return #NAME;
        ENABLE_DISABLE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ENABLE_DISABLE_LIST

//--------------------------------------
//  Direction Settings
//--------------------------------------
/**
 * @brief Direction settings used by various encoder parameters.
 *
 * Table — Direction Settings:
 *  NUMBER | NAME          | DESCRIPTION
 *  ------ | ------------- | -----------------------------------------------
 *     0   | NOT_INVERTED  | Normal direction.
 *     1   | INVERTED      | Inverted direction.
 */
#define DIRECTION_LIST(X) \
    X(NOT_INVERTED, 0, /*!< Normal direction. */) \
    X(INVERTED,     1, /*!< Inverted direction. */)

enum class Direction : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    DIRECTION_LIST(X)
    #undef X
};

inline const char* to_string(Direction direction) {
    switch(direction) {
        #define X(NAME, VALUE, DOC) case Direction::NAME: return #NAME;
        DIRECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef DIRECTION_LIST

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ████████╗ ██████╗ ██████╗  ██████╗ ██╗   ██╗███████╗     █████╗ ███╗   ██╗██████╗                             //
//    ╚══██╔══╝██╔═══██╗██╔══██╗██╔═══██╗██║   ██║██╔════╝    ██╔══██╗████╗  ██║██╔══██╗                            //
//       ██║   ██║   ██║██████╔╝██║   ██║██║   ██║█████╗      ███████║██╔██╗ ██║██║  ██║                            //
//       ██║   ██║   ██║██╔══██╗██║▄▄ ██║██║   ██║██╔══╝      ██╔══██║██║╚██╗██║██║  ██║                            //
//       ██║   ╚██████╔╝██║  ██║╚██████╔╝╚██████╔╝███████╗    ██║  ██║██║ ╚████║██████╔╝                            //
//       ╚═╝    ╚═════╝ ╚═╝  ╚═╝ ╚══▀▀═╝  ╚═════╝ ╚══════╝    ╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝                             //
//                                                                                                                  //
//    ███████╗██╗     ██╗   ██╗██╗  ██╗     ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗                 //
//    ██╔════╝██║     ██║   ██║╚██╗██╔╝    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║                 //
//    █████╗  ██║     ██║   ██║ ╚███╔╝     ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║                 //
//    ██╔══╝  ██║     ██║   ██║ ██╔██╗     ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║                 //
//    ██║     ███████╗╚██████╔╝██╔╝ ██╗    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗            //
//    ╚═╝     ╚══════╝ ╚═════╝ ╚═╝  ╚═╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝            //
//                                                                                                                  //
//==================================================================================================================//
//                                  TORQUE AND FLUX CONTROL SECTION                                                 // 
//==================================================================================================================//
//==================================================================================================================//

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Torque and Flux Control Parameters
/// @{
//--------------------------------------
//  Torque and Flux Control Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring torque and flux control loops.
 *
 * Table — Torque and Flux Control Parameters:
 *  NUMBER | NAME                              | DESCRIPTION
 *  ------ | --------------------------------- | -----------------------------------------------------------------
 *     6   | MAX_TORQUE                        | Maximum motor torque [mA]. 0...65535. Default: 2000. RWE
 *     7   | MAX_FLUX                          | Maximum motor flux [mA]. 0...65535. Default: 2000. RWE
 *   104   | TARGET_TORQUE                     | Target torque [mA]. Write to activate torque regulation. -32768...32767. Default: 0. RW
 *   105   | ACTUAL_TORQUE                     | Actual motor torque [mA]. -32767...32768. Default: 0. R
 *   106   | TARGET_FLUX                       | Target flux [mA]. -10000...10000. Default: 0. RW
 *   107   | ACTUAL_FLUX                       | Actual motor flux [mA]. -2147483648...2147483647. Default: 0. R
 *   108   | TORQUE_OFFSET                     | Offset applied to torque value [mA]. -4700...4700. Default: 0. RW
 *   109   | TORQUE_P                          | P parameter for torque PI regulator. 0...32767. Default: 50. RWE
 *   110   | TORQUE_I                          | I parameter for torque PI regulator. 0...32767. Default: 100. RWE
 *   111   | FLUX_P                            | P parameter for flux PI regulator (if separated). 0...32767. Default: 50. RWE
 *   112   | FLUX_I                            | I parameter for flux PI regulator (if separated). 0...32767. Default: 100. RWE
 *   113   | SEPARATE_TORQUE_FLUX_PI_PARAMETERS| Enable separate PI values for torque/flux. 0: COMBINED, 1: SEPARATED. Default: 0. RWE
 *   114   | CURRENT_NORM_P                    | P normalization format for current PI. 0: SHIFT_8_BIT, 1: SHIFT_16_BIT. Default: 0. RWE
 *   115   | CURRENT_NORM_I                    | I normalization format for current PI. 0: SHIFT_8_BIT, 1: SHIFT_16_BIT. Default: 1. RWE
 *   116   | TORQUE_PI_ERROR                   | Torque PI regulator error. -2147483648...2147483647. Default: 0. R
 *   117   | FLUX_PI_ERROR                     | Flux PI regulator error. -2147483648...2147483647. Default: 0. R
 *   118   | TORQUE_PI_INTEGRATOR              | Integrated error of torque PI. -2147483648...2147483647. Default: 0. R
 *   119   | FLUX_PI_INTEGRATOR                | Integrated error of flux PI. -2147483648...2147483647. Default: 0. R
 *   120   | FLUX_OFFSET                       | Offset applied to flux value [mA]. -4700...4700. Default: 0. RW
 *   308   | FIELDWEAKENING_I                  | I parameter for field weakening controller. 0...32767. Default: 0. RWE
 *   310   | FIELDWEAKENING_VOLTAGE_THRESHOLD  | Max voltage for field weakening. 0...32767. Default: 32767. RWE
 */
#define TORQUE_FLUX_CONTROL_LIST(X) \
    X(MAX_TORQUE,                        6,   /*!< Maximum motor torque [mA]. 0...65535. Default: 2000. RWE */) \
    X(MAX_FLUX,                          7,   /*!< Maximum motor flux [mA]. 0...65535. Default: 2000. RWE */) \
    X(TARGET_TORQUE,                    104,  /*!< Target torque [mA]. Write to activate torque regulation. -32768...32767. Default: 0. RW */) \
    X(ACTUAL_TORQUE,                    105,  /*!< Actual motor torque [mA]. -32767...32768. Default: 0. R */) \
    X(TARGET_FLUX,                      106,  /*!< Target flux [mA]. -10000...10000. Default: 0. RW */) \
    X(ACTUAL_FLUX,                      107,  /*!< Actual motor flux [mA]. -2147483648...2147483647. Default: 0. R */) \
    X(TORQUE_OFFSET,                    108,  /*!< Offset applied to torque value [mA]. -4700...4700. Default: 0. RW */) \
    X(TORQUE_P,                         109,  /*!< P parameter for torque PI regulator. 0...32767. Default: 50. RWE */) \
    X(TORQUE_I,                         110,  /*!< I parameter for torque PI regulator. 0...32767. Default: 100. RWE */) \
    X(FLUX_P,                           111,  /*!< P parameter for flux PI regulator (if separated). 0...32767. Default: 50. RWE */) \
    X(FLUX_I,                           112,  /*!< I parameter for flux PI regulator (if separated). 0...32767. Default: 100. RWE */) \
    X(SEPARATE_TORQUE_FLUX_PI_PARAMETERS,113, /*!< Enable separate PI values for torque/flux. 0: COMBINED, 1: SEPARATED. Default: 0. RWE */) \
    X(CURRENT_NORM_P,                   114,  /*!< P normalization format for current PI. 0: SHIFT_8_BIT, 1: SHIFT_16_BIT. Default: 0. RWE */) \
    X(CURRENT_NORM_I,                   115,  /*!< I normalization format for current PI. 0: SHIFT_8_BIT, 1: SHIFT_16_BIT. Default: 1. RWE */) \
    X(TORQUE_PI_ERROR,                  116,  /*!< Torque PI regulator error. -2147483648...2147483647. Default: 0. R */) \
    X(FLUX_PI_ERROR,                    117,  /*!< Flux PI regulator error. -2147483648...2147483647. Default: 0. R */) \
    X(TORQUE_PI_INTEGRATOR,             118,  /*!< Integrated error of torque PI. -2147483648...2147483647. Default: 0. R */) \
    X(FLUX_PI_INTEGRATOR,               119,  /*!< Integrated error of flux PI. -2147483648...2147483647. Default: 0. R */) \
    X(FLUX_OFFSET,                      120,  /*!< Offset applied to flux value [mA]. -4700...4700. Default: 0. RW */) \
    X(FIELDWEAKENING_I,                 308,  /*!< I parameter for field weakening controller. 0...32767. Default: 0. RWE */) \
    X(FIELDWEAKENING_VOLTAGE_THRESHOLD, 310,  /*!< Max voltage for field weakening. 0...32767. Default: 32767. RWE */)

enum class TorqueFluxControl : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    TORQUE_FLUX_CONTROL_LIST(X)
    #undef X
};

inline const char* to_string(TorqueFluxControl config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case TorqueFluxControl::NAME: return #NAME;
        TORQUE_FLUX_CONTROL_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Torque/Flux PI Separation
/// @{
//--------------------------------------
//  Torque/Flux PI Separation
//--------------------------------------
/**
 * @brief Selects if torque and flux PI controllers use separate parameters.
 *
 * Table — Torque/Flux PI Separation:
 *  NUMBER | NAME                     | DESCRIPTION
 *  ------ | ------------------------ | -----------------------------------------------------------------
 *     0   | TORQUE_FLUX_PI_COMBINED  | Use same PI parameters for torque and flux.
 *     1   | TORQUE_FLUX_PI_SEPARATED | Use separate PI parameters for torque and flux.
 */
#define TORQUE_FLUX_PI_SEPARATION_LIST(X) \
    X(TORQUE_FLUX_PI_COMBINED,  0, /*!< Use same PI parameters for torque and flux. */) \
    X(TORQUE_FLUX_PI_SEPARATED, 1, /*!< Use separate PI parameters for torque and flux. */)

enum class TorqueFluxPiSeparation : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    TORQUE_FLUX_PI_SEPARATION_LIST(X)
    #undef X
};

inline const char* to_string(TorqueFluxPiSeparation separation) {
    switch(separation) {
        #define X(NAME, VALUE, DOC) case TorqueFluxPiSeparation::NAME: return #NAME;
        TORQUE_FLUX_PI_SEPARATION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef TORQUE_FLUX_PI_SEPARATION_LIST
/// @}

/// @name Current PI Normalization Format
/// @{
//--------------------------------------
//  Current PI Normalization Format
//--------------------------------------
/**
 * @brief Normalization format for current PI controller output.
 *
 * Table — Current PI Normalization Format:
 *  NUMBER | NAME        | DESCRIPTION
 *  ------ | ----------- | -----------------------------------------------
 *     0   | SHIFT_8_BIT | Output shifted right by 8 bits.
 *     1   | SHIFT_16_BIT| Output shifted right by 16 bits.
 */
#define CURRENT_PI_NORMALIZATION_LIST(X) \
    X(SHIFT_8_BIT,  0, /*!< Output shifted right by 8 bits. */) \
    X(SHIFT_16_BIT, 1, /*!< Output shifted right by 16 bits. */)

enum class CurrentPiNormalization : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    CURRENT_PI_NORMALIZATION_LIST(X)
    #undef X
};

inline const char* to_string(CurrentPiNormalization norm) {
    switch(norm) {
        #define X(NAME, VALUE, DOC) case CurrentPiNormalization::NAME: return #NAME;
        CURRENT_PI_NORMALIZATION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef CURRENT_PI_NORMALIZATION_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗   ██╗███████╗██╗      ██████╗  ██████╗██╗████████╗██╗   ██╗    ███╗   ███╗ ██████╗ ██████╗ ███████╗       //
//    ██║   ██║██╔════╝██║     ██╔═══██╗██╔════╝██║╚══██╔══╝╚██╗ ██╔╝    ████╗ ████║██╔═══██╗██╔══██╗██╔════╝       //
//    ██║   ██║█████╗  ██║     ██║   ██║██║     ██║   ██║    ╚████╔╝     ██╔████╔██║██║   ██║██║  ██║█████╗         //
//    ╚██╗ ██╔╝██╔══╝  ██║     ██║   ██║██║     ██║   ██║     ╚██╔╝      ██║╚██╔╝██║██║   ██║██║  ██║██╔══╝         //
//     ╚████╔╝ ███████╗███████╗╚██████╔╝╚██████╗██║   ██║      ██║       ██║ ╚═╝ ██║╚██████╔╝██████╔╝███████╗       //
//      ╚═══╝  ╚══════╝╚══════╝ ╚═════╝  ╚═════╝╚═╝   ╚═╝      ╚═╝       ╚═╝     ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝       //
//                                                                                                                  //
//==================================================================================================================//
//                                        VELOCITY MODE SECTION                                                     // 
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Velocity Mode Parameters
/// @{
//--------------------------------------
//  Velocity Control Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring velocity control and ramping.
 *
 * Table — Velocity Control Parameters:
 *  NUMBER | NAME                          | DESCRIPTION
 *  ------ | ----------------------------- | -----------------------------------------------------------------
 *   123   | VELOCITY_SENSOR_SELECTION     | Feedback source for velocity PI regulator. See VelocitySensorSelection. Default: 0 (SAME_AS_COMMUTATION). RWE
 *   124   | TARGET_VELOCITY               | Target velocity value. Write to activate velocity regulation. -134217728...134217727. Default: 0. RW
 *   125   | ACTUAL_VELOCITY               | Actual velocity value. -2147483648...2147483647. Default: 0. R
 *   126   | VELOCITY_OFFSET               | Offset applied to velocity value. -200000...200000. Default: 0. RW 
 *   127   | VELOCITY_P                    | P parameter for velocity PI regulator. 0...32767. Default: 800. RWE
 *   128   | VELOCITY_I                    | I parameter for velocity PI regulator. 0...32767. Default: 1. RWE
 *   129   | VELOCITY_NORM_P               | P normalization for velocity PI. See VelocityPiNorm. Default: 2. RWE
 *   130   | VELOCITY_NORM_I               | I normalization for velocity PI. See VelocityPiNorm. Default: 2. RWE
 *   131   | VELOCITY_PI_INTEGRATOR        | Integrated error of velocity PI regulator. -2147483648...2147483647. Default: 0. R
 *   132   | VELOCITY_PI_ERROR             | Velocity PI regulator error. -2147483648...2147483647. Default: 0. R
 *   133   | VELOCITY_SCALING_FACTOR       | Scaling factor for velocity to real-world units. 1...2047. Default: 1. RWE
 *   135   | VELOCITY_LOOP_DOWNSAMPLING    | Downsampling factor for velocity controller. 0...127. Default: 5. RWE
 *   137   | VELOCITY_METER_SWITCH_THRESHOLD | Threshold for switching from period to frequency velocity meter. 0...134217727. Default: 2000. RWE
 *   138   | VELOCITY_METER_SWITCH_HYSTERESIS | Hysteresis for switching back to period meter. 0...65535. Default: 500. RWE
 *   139   | VELOCITY_METER_MODE           | Currently used velocity meter mode. See VelocityMeterMode. Default: 0. R
 *    45   | OPENLOOP_ANGLE                | Phi_e calculated by ramper hardware (openloop modes). -32768...32767. Default: 0. R
 *    46   | OPENLOOP_CURRENT              | Openloop current applied in openloop, current mode [mA]. 0...65535. Default: 1000. RWE
 *    47   | OPENLOOP_VOLTAGE              | Openloop voltage applied in openloop, voltage mode. 0...16383. Default: 0. RWE
 *    50   | ACCELERATION_FF_GAIN          | Gain for acceleration feedforward. 0...65535. Default: 8. RWE
 *    51   | ACCELERATION_FF_SHIFT         | Shift for acceleration feedforward. See AccelerationFfShift. Default: 4. RWE
 *    52   | RAMP_ENABLE                   | Enable acceleration/deceleration ramps. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 *    53   | DIRECT_VELOCITY_MODE          | What the velocity PI tracks. 0: RAMP_VELOCITY (open-loop ramper / φe). 1: TARGET_VELOCITY (closed-loop FOC). Default: 1. RWE
 *    54   | RAMP_AMAX                     | Max acceleration (top part of ramp). 1...8388607. Default: 1000. RWE
 *    55   | RAMP_A1                       | First acceleration in ramp. 1...8388607. Default: 8000. RWE
 *    56   | RAMP_A2                       | Second acceleration in ramp. 1...8388607. Default: 4000. RWE
 *    57   | RAMP_DMAX                     | Max deceleration (top part of ramp). 1...8388607. Default: 1000. RWE
 *    58   | RAMP_D1                       | Second deceleration in ramp. 1...8388607. Default: 8000. RWE
 *    59   | RAMP_D2                       | First deceleration in ramp. 1...8388607. Default: 8000. RWE
 *    60   | RAMP_VMAX                     | Max velocity of ramp. 0...134217727. Default: 134217727. RWE
 *    61   | RAMP_V1                       | Velocity threshold for A1/D1 to A2/D2. 0...134217727. Default: 0. RWE
 *    62   | RAMP_V2                       | Velocity threshold for A2/D2 to AMAX/DMAX. 0...134217727. Default: 0. RWE
 *    63   | RAMP_VSTART                   | Start velocity of ramp. 0...8388607. Default: 0. RWE
 *    64   | RAMP_VSTOP                    | Stop velocity of ramp. 1...8388607. Default: 1. RWE
 *    65   | RAMP_TVMAX                    | Min time at VMAX before deceleration. 0...65535. Default: 0. RWE
 *    66   | RAMP_TZEROWAIT                | Wait time at end of ramp. 0...65535. Default: 0. RWE
 *    67   | ACCELERATION_FEEDFORWARD_ENABLE | Enable acceleration feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 *    68   | VELOCITY_FEEDFORWARD_ENABLE   | Enable velocity feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 *    69   | RAMP_VELOCITY                 | Target velocity calculated by ramp controller. -134217727...134217727. Default: 0. R
 *    70   | RAMP_VELOCITY_ERROR            | Error of ramp velocity controller. -134217727...134217727. Default: 0. R
 */
#define VELOCITY_CONTROL_LIST(X) \
    X(VELOCITY_SENSOR_SELECTION,          123, /*!< Feedback source for velocity PI regulator. See VelocitySensorSelection. Default: 0 (SAME_AS_COMMUTATION). RWE */) \
    X(TARGET_VELOCITY,                    124, /*!< Target velocity value. Write to activate velocity regulation. -134217728...134217727. Default: 0. RW */) \
    X(ACTUAL_VELOCITY,                    125, /*!< Actual velocity value. -2147483648...2147483647. Default: 0. R */) \
    X(VELOCITY_OFFSET,                    126, /*!< Offset applied to velocity value. -200000...200000. Default: 0. RW */) \
    X(VELOCITY_P,                         127, /*!< P parameter for velocity PI regulator. 0...32767. Default: 800. RWE */) \
    X(VELOCITY_I,                         128, /*!< I parameter for velocity PI regulator. 0...32767. Default: 1. RWE */) \
    X(VELOCITY_NORM_P,                    129, /*!< P normalization for velocity PI. See VelocityPiNorm. Default: 2. RWE */) \
    X(VELOCITY_NORM_I,                    130, /*!< I normalization for velocity PI. See VelocityPiNorm. Default: 2. RWE */) \
    X(VELOCITY_PI_INTEGRATOR,             131, /*!< Integrated error of velocity PI regulator. -2147483648...2147483647. Default: 0. R */) \
    X(VELOCITY_PI_ERROR,                  132, /*!< Velocity PI regulator error. -2147483648...2147483647. Default: 0. R */) \
    X(VELOCITY_SCALING_FACTOR,            133, /*!< Scaling factor for velocity to real-world units. 1...2047. Default: 1. RWE */) \
    X(VELOCITY_LOOP_DOWNSAMPLING,         135, /*!< Downsampling factor for velocity controller. 0...127. Default: 5. RWE */) \
    X(VELOCITY_METER_SWITCH_THRESHOLD,    137, /*!< Threshold for switching from period to frequency velocity meter. 0...134217727. Default: 2000. RWE */) \
    X(VELOCITY_METER_SWITCH_HYSTERESIS,   138, /*!< Hysteresis for switching back to period meter. 0...65535. Default: 500. RWE */) \
    X(VELOCITY_METER_MODE,                139, /*!< Currently used velocity meter mode. See VelocityMeterMode. Default: 0. R */) \
    X(OPENLOOP_ANGLE,                      45, /*!< Phi_e calculated by ramper hardware (openloop modes). -32768...32767. Default: 0. R */) \
    X(OPENLOOP_CURRENT,                    46, /*!< Openloop current applied in openloop, current mode [mA]. 0...65535. Default: 1000. RWE */) \
    X(OPENLOOP_VOLTAGE,                    47, /*!< Openloop voltage applied in openloop, voltage mode. 0...16383. Default: 0. RWE */) \
    X(ACCELERATION_FF_GAIN,                50, /*!< Gain for acceleration feedforward. 0...65535. Default: 8. RWE */) \
    X(ACCELERATION_FF_SHIFT,               51, /*!< Shift for acceleration feedforward. See AccelerationFfShift. Default: 4. RWE */) \
    X(RAMP_ENABLE,                         52, /*!< Enable acceleration/deceleration ramps. 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(DIRECT_VELOCITY_MODE,                53, /*!< 0: PI tracks RAMP_VELOCITY (open-loop ramper). 1: PI tracks TARGET_VELOCITY (closed-loop FOC). Default: 1. RWE */) \
    X(RAMP_AMAX,                           54, /*!< Max acceleration (top part of ramp). 1...8388607. Default: 1000. RWE */) \
    X(RAMP_A1,                             55, /*!< First acceleration in ramp. 1...8388607. Default: 8000. RWE */) \
    X(RAMP_A2,                             56, /*!< Second acceleration in ramp. 1...8388607. Default: 4000. RWE */) \
    X(RAMP_DMAX,                           57, /*!< Max deceleration (top part of ramp). 1...8388607. Default: 1000. RWE */) \
    X(RAMP_D1,                             58, /*!< Second deceleration in ramp. 1...8388607. Default: 8000. RWE */) \
    X(RAMP_D2,                             59, /*!< First deceleration in ramp. 1...8388607. Default: 8000. RWE */) \
    X(RAMP_VMAX,                           60, /*!< Max velocity of ramp. 0...134217727. Default: 134217727. RWE */) \
    X(RAMP_V1,                             61, /*!< Velocity threshold for A1/D1 to A2/D2. 0...134217727. Default: 0. RWE */) \
    X(RAMP_V2,                             62, /*!< Velocity threshold for A2/D2 to AMAX/DMAX. 0...134217727. Default: 0. RWE */) \
    X(RAMP_VSTART,                         63, /*!< Start velocity of ramp. 0...8388607. Default: 0. RWE */) \
    X(RAMP_VSTOP,                          64, /*!< Stop velocity of ramp. 1...8388607. Default: 1. RWE */) \
    X(RAMP_TVMAX,                          65, /*!< Min time at VMAX before deceleration. 0...65535. Default: 0. RWE */) \
    X(RAMP_TZEROWAIT,                      66, /*!< Wait time at end of ramp. 0...65535. Default: 0. RWE */) \
    X(ACCELERATION_FEEDFORWARD_ENABLE,     67, /*!< Enable acceleration feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(VELOCITY_FEEDFORWARD_ENABLE,         68, /*!< Enable velocity feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(RAMP_VELOCITY,                       69, /*!< Target velocity calculated by ramp controller. -134217727...134217727. Default: 0. R */) \
    X(RAMP_POSITION,                       70, /*!< Target position calculated by ramp controller. -2147483648...2147483647. Default: 0. R */) \

enum class VelocityControl : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VELOCITY_CONTROL_LIST(X)
    #undef X
};

inline const char* to_string(VelocityControl config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case VelocityControl::NAME: return #NAME;
        VELOCITY_CONTROL_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Velocity Sensor Selection
/// @{
//--------------------------------------
//  Velocity Sensor Selection
//--------------------------------------
/**
 * @brief Enumerates feedback sources for velocity PI regulator.
 *
 * Table — Velocity Sensor Selection:
 *  NUMBER | NAME          | DESCRIPTION
 *  ------ | ------------- | -----------------------------------------------
 *     0   | SAME_AS_COMMUTATION | Use same feedback as commutation mode.
 *     1   | DIGITAL_HALL        | Use digital Hall sensors.
 *     2   | ABN1_ENCODER        | Use ABN1 encoder.
 *     3   | ABN2_ENCODER        | Use ABN2 encoder.
 *     4   | SPI_ENCODER         | Use SPI encoder.
 */
#define VELOCITY_SENSOR_SELECTION_LIST(X) \
    X(SAME_AS_COMMUTATION, 0, /*!< Use same feedback as commutation mode. */) \
    X(DIGITAL_HALL,        1, /*!< Use digital Hall sensors. */) \
    X(ABN1_ENCODER,        2, /*!< Use ABN1 encoder. */) \
    X(ABN2_ENCODER,        3, /*!< Use ABN2 encoder. */) \
    X(SPI_ENCODER,         4, /*!< Use SPI encoder. */)

enum class VelocitySensorSelection : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VELOCITY_SENSOR_SELECTION_LIST(X)
    #undef X
};

inline const char* to_string(VelocitySensorSelection selection) {
    switch(selection) {
        #define X(NAME, VALUE, DOC) case VelocitySensorSelection::NAME: return #NAME;
        VELOCITY_SENSOR_SELECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VELOCITY_SENSOR_SELECTION_LIST
/// @}

/// @name Velocity PI Normalization
/// @{
//--------------------------------------
//  Velocity PI Normalization
//--------------------------------------
/**
 * @brief Enumerates normalization formats for velocity PI controller.
 *
 * Table — Velocity PI Normalization:
 *  NUMBER | NAME        | DESCRIPTION
 *  ------ | ----------- | -----------------------------------------------
 *     0   | NO_SHIFT    | No shift.
 *     1   | SHIFT_8_BIT | Shift right by 8 bits.
 *     2   | SHIFT_16_BIT| Shift right by 16 bits.
 *     3   | SHIFT_24_BIT| Shift right by 24 bits.
 */
#define VELOCITY_PI_NORM_LIST(X) \
    X(NO_SHIFT,    0, /*!< No shift. */) \
    X(SHIFT_8_BIT, 1, /*!< Shift right by 8 bits. */) \
    X(SHIFT_16_BIT,2, /*!< Shift right by 16 bits. */) \
    X(SHIFT_24_BIT,3, /*!< Shift right by 24 bits. */)

enum class VelocityPiNorm : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VELOCITY_PI_NORM_LIST(X)
    #undef X
};

inline const char* to_string(VelocityPiNorm norm) {
    switch(norm) {
        #define X(NAME, VALUE, DOC) case VelocityPiNorm::NAME: return #NAME;
        VELOCITY_PI_NORM_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VELOCITY_PI_NORM_LIST
/// @}

/// @name Velocity Meter Modes
/// @{
//--------------------------------------
//  Velocity Meter Modes
//--------------------------------------
/**
 * @brief Enumerates velocity meter modes.
 *
 * Table — Velocity Meter Modes:
 *  NUMBER | NAME            | DESCRIPTION
 *  ------ | --------------- | -----------------------------------------------
 *     0   | PERIOD_METER    | Period-based measurement.
 *     1   | FREQUENCY_METER | Frequency-based measurement.
 *     2   | SOFTWARE_METER  | Software-based measurement.
 */
#define VELOCITY_METER_MODE_LIST(X) \
    X(PERIOD_METER,    0, /*!< Period-based measurement. */) \
    X(FREQUENCY_METER, 1, /*!< Frequency-based measurement. */) \
    X(SOFTWARE_METER,  2, /*!< Software-based measurement. */)

enum class VelocityMeterMode : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    VELOCITY_METER_MODE_LIST(X)
    #undef X
};

inline const char* to_string(VelocityMeterMode mode) {
    switch(mode) {
        #define X(NAME, VALUE, DOC) case VelocityMeterMode::NAME: return #NAME;
        VELOCITY_METER_MODE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef VELOCITY_METER_MODE_LIST
/// @}

/// @name Acceleration Feedforward Shift
/// @{
//--------------------------------------
//  Acceleration Feedforward Shift
//--------------------------------------
/**
 * @brief Enumerates shift values for acceleration feedforward.
 *
 * Table — Acceleration Feedforward Shift:
 *  NUMBER | NAME        | DESCRIPTION
 *  ------ | ----------- | -----------------------------------------------
 *     0   | NO_SHIFT    | No shift.
 *     1   | SHIFT_4_BIT | Shift right by 4 bits.
 *     2   | SHIFT_8_BIT | Shift right by 8 bits.
 *     3   | SHIFT_12_BIT| Shift right by 12 bits.
 *     4   | SHIFT_16_BIT| Shift right by 16 bits.
 *     5   | SHIFT_20_BIT| Shift right by 20 bits.
 *     6   | SHIFT_24_BIT| Shift right by 24 bits.
 */
#define ACCELERATION_FF_SHIFT_LIST(X) \
    X(NO_SHIFT,     0, /*!< No shift. */) \
    X(SHIFT_4_BIT,  1, /*!< Shift right by 4 bits. */) \
    X(SHIFT_8_BIT,  2, /*!< Shift right by 8 bits. */) \
    X(SHIFT_12_BIT, 3, /*!< Shift right by 12 bits. */) \
    X(SHIFT_16_BIT, 4, /*!< Shift right by 16 bits. */) \
    X(SHIFT_20_BIT, 5, /*!< Shift right by 20 bits. */) \
    X(SHIFT_24_BIT, 6,  /*!< Shift right by 24 bits. */)

enum class AccelerationFFShift : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ACCELERATION_FF_SHIFT_LIST(X)
    #undef X
};

inline const char* to_string(AccelerationFFShift shift) {
    switch(shift) {
        #define X(NAME, VALUE, DOC) case AccelerationFFShift::NAME: return #NAME;
        ACCELERATION_FF_SHIFT_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ACCELERATION_FF_SHIFT_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗  ██████╗ ███████╗██╗████████╗██╗ ██████╗ ███╗   ██╗    ███╗   ███╗ ██████╗ ██████╗ ███████╗           //
//    ██╔══██╗██╔═══██╗██╔════╝██║╚══██╔══╝██║██╔═══██╗████╗  ██║    ████╗ ████║██╔═══██╗██╔══██╗██╔════╝           //
//    ██████╔╝██║   ██║███████╗██║   ██║   ██║██║   ██║██╔██╗ ██║    ██╔████╔██║██║   ██║██║  ██║█████╗             //
//    ██╔═══╝ ██║   ██║╚════██║██║   ██║   ██║██║   ██║██║╚██╗██║    ██║╚██╔╝██║██║   ██║██║  ██║██╔══╝             //
//    ██║     ╚██████╔╝███████║██║   ██║   ██║╚██████╔╝██║ ╚████║    ██║ ╚═╝ ██║╚██████╔╝██████╔╝███████╗           //
//    ╚═╝      ╚═════╝ ╚══════╝╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝    ╚═╝     ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝           //
//                                                                                                                  //
//==================================================================================================================//
//                                         POSITION MODE SECTION                                                    // 
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Position Mode Parameters
/// @{
//--------------------------------------
//  Position Control Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring position control and ramping.
 *
 * Table — Position Control Parameters:
 *  NUMBER | NAME                  | DESCRIPTION
 *  ------ | --------------------- | -----------------------------------------------------------------
 *   142   | POSITION_SENSOR_SELECTION | Feedback source for position PI regulator. See PositionSensorSelection. Default: 0 (SAME_AS_COMMUTATION). RWE
 *   143   | TARGET_POSITION           | Target position value. Write to activate position regulation. -2147483648...2147483647. Default: 0. RW
 *   144   | ACTUAL_POSITION           | Actual position value. -2147483648...2147483647. Default: 0. RW
 *   145   | POSITION_SCALING_FACTOR   | Scaling factor for position to real-world units. 1024...65535. Default: 1024. RWE
 *   146   | POSITION_P                | P parameter for position PI regulator. 0...32767. Default: 5. RWE
 *   147   | POSITION_I                | I parameter for position PI regulator. 0...32767. Default: 0. RWE
 *   148   | POSITION_NORM_P           | P normalization for position PI. See PositionPiNorm. Default: 1. RWE
 *   149   | POSITION_NORM_I           | I normalization for position PI. See PositionPiNorm. Default: 1. RWE
 *   150   | POSITION_PI_INTEGRATOR    | Integrated error of position PI regulator. -2147483648...2147483647. Default: 0. R
 *   151   | POSITION_PI_ERROR         | Error of position PI regulator. -2147483648...2147483647. Default: 0. R
 *   153   | POSITION_LOOP_DOWNSAMPLING| Downsampling factor for position controller. 0...127. Default: 0. RWE
 *   154   | LATCH_POSITION            | Position switch latched. -2147483648...2147483647. Default: 0. R
 *   155   | POSITION_LIMIT_LOW        | Position limit low. -2147483648...2147483647. Default: -2147483648. RWE
 *   156   | POSITION_LIMIT_HIGH       | Position limit high. -2147483648...2147483647. Default: 2147483647. RWE
 *   157   | POSITION_REACHED_THRESHOLD| Deviation between target and actual position below which the position reached flag goes active and latches. If a new target position is set the flag is reset. 0...2147483647. Default: 0. RWE
 */
#define POSITION_CONTROL_LIST(X) \
    X(POSITION_SENSOR_SELECTION, 142, /*!< Feedback source for position PI regulator. See PositionSensorSelection. Default: 0 (SAME_AS_COMMUTATION). RWE */) \
    X(TARGET_POSITION,           143, /*!< Target position value. Write to activate position regulation. -2147483648...2147483647. Default: 0. RW */) \
    X(ACTUAL_POSITION,           144, /*!< Actual position value. -2147483648...2147483647. Default: 0. RW */) \
    X(POSITION_SCALING_FACTOR,   145, /*!< Scaling factor for position to real-world units. 1024...65535. Default: 1024. RWE */) \
    X(POSITION_P,                146, /*!< P parameter for position PI regulator. 0...32767. Default: 5. RWE */) \
    X(POSITION_I,                147, /*!< I parameter for position PI regulator. 0...32767. Default: 0. RWE */) \
    X(POSITION_NORM_P,           148, /*!< P normalization for position PI. See PositionPiNorm. Default: 1. RWE */) \
    X(POSITION_NORM_I,           149, /*!< I normalization for position PI. See PositionPiNorm. Default: 1. RWE */) \
    X(POSITION_PI_INTEGRATOR,    150, /*!< Integrated error of position PI regulator. -2147483648...2147483647. Default: 0. R */) \
    X(POSITION_PI_ERROR,         151, /*!< Error of position PI regulator. -2147483648...2147483647. Default: 0. R */) \
    X(POSITION_LOOP_DOWNSAMPLING,153, /*!< Downsampling factor for position controller. 0...127. Default: 0. RWE */) \
    X(LATCH_POSITION,            154, /*!< Position switch latched. -2147483648...2147483647. Default: 0. R */) \
    X(POSITION_LIMIT_LOW,        155, /*!< Position limit low. -2147483648...2147483647. Default: -2147483648. RWE */) \
    X(POSITION_LIMIT_HIGH,       156, /*!< Position limit high. -2147483648...2147483647. Default: 2147483647. RWE */) \
    X(POSITION_REACHED_THRESHOLD,157, /*!< Deviation between target and actual position below which the position reached flag goes active and latches. If a new target position is set the flag is reset. 0...2147483647. Default: 0. RWE */)

enum class PositionControl : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    POSITION_CONTROL_LIST(X)
    #undef X
};

inline const char* to_string(PositionControl config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case PositionControl::NAME: return #NAME;
        POSITION_CONTROL_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Position Sensor Selection
/// @{
//--------------------------------------
//  Position Sensor Selection
//--------------------------------------
/**
 * @brief Enumerates feedback sources for position PI regulator.
 *
 * Table — Position Sensor Selection:
 *  NUMBER | NAME              | DESCRIPTION
 *  ------ | ----------------- | -----------------------------------------------
 *     0   | SAME_AS_COMMUTATION | Use same feedback as commutation mode.
 *     1   | DIGITAL_HALL        | Use digital Hall sensors.
 *     2   | ABN1_ENCODER        | Use ABN1 encoder.
 *     3   | ABN2_ENCODER        | Use ABN2 encoder.
 *     4   | SPI_ENCODER         | Use SPI encoder.
 */
#define POSITION_SENSOR_SELECTION_LIST(X) \
    X(SAME_AS_COMMUTATION, 0, /*!< Use same feedback as commutation mode. */) \
    X(DIGITAL_HALL,        1, /*!< Use digital Hall sensors. */) \
    X(ABN1_ENCODER,        2, /*!< Use ABN1 encoder. */) \
    X(ABN2_ENCODER,        3, /*!< Use ABN2 encoder. */) \
    X(SPI_ENCODER,         4, /*!< Use SPI encoder. */)

enum class PositionSensorSelection : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    POSITION_SENSOR_SELECTION_LIST(X)
    #undef X
};

inline const char* to_string(PositionSensorSelection selection) {
    switch(selection) {
        #define X(NAME, VALUE, DOC) case PositionSensorSelection::NAME: return #NAME;
        POSITION_SENSOR_SELECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef POSITION_SENSOR_SELECTION_LIST
/// @}

/// @name Position PI Normalization
/// @{
//--------------------------------------
//  Position PI Normalization
//--------------------------------------
/**
 * @brief Enumerates normalization formats for position PI controller.
 *
 * Table — Position PI Normalization:
 *  NUMBER | NAME        | DESCRIPTION
 *  ------ | ----------- | -----------------------------------------------
 *     0   | NO_SHIFT    | No shift.
 *     1   | SHIFT_8_BIT | Shift right by 8 bits.
 *     2   | SHIFT_16_BIT| Shift right by 16 bits.
 *     3   | SHIFT_24_BIT| Shift right by 24 bits.
 */
#define POSITION_PI_NORM_LIST(X) \
    X(NO_SHIFT,     0, /*!< No shift. */) \
    X(SHIFT_8_BIT,  1, /*!< Shift right by 8 bits. */) \
    X(SHIFT_16_BIT, 2, /*!< Shift right by 16 bits. */) \
    X(SHIFT_24_BIT, 3, /*!< Shift right by 24 bits. */)

enum class PositionPiNorm : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    POSITION_PI_NORM_LIST(X)
    #undef X
};

inline const char* to_string(PositionPiNorm norm) {
    switch(norm) {
        #define X(NAME, VALUE, DOC) case PositionPiNorm::NAME: return #NAME;
        POSITION_PI_NORM_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef POSITION_PI_NORM_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗  █████╗ ███╗   ███╗██████╗ ███████╗██████╗     ███████╗████████╗ ██████╗ ██████╗                      //
//    ██╔══██╗██╔══██╗████╗ ████║██╔══██╗██╔════╝██╔══██╗    ██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗                     //
//    ██████╔╝███████║██╔████╔██║██████╔╝█████╗  ██████╔╝    ███████╗   ██║   ██║   ██║██████╔╝                     //
//    ██╔══██╗██╔══██║██║╚██╔╝██║██╔═══╝ ██╔══╝  ██╔══██╗    ╚════██║   ██║   ██║   ██║██╔═══╝                      //
//    ██║  ██║██║  ██║██║ ╚═╝ ██║██║     ███████╗██║  ██║    ███████║   ██║   ╚██████╔╝██║                          //
//    ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝    ╚══════╝   ╚═╝    ╚═════╝ ╚═╝                          //
//                                                                                                                  //
//     ██████╗ ██████╗ ███╗   ██╗██████╗ ██╗████████╗██╗ ██████╗ ███╗   ██╗███████╗      ██╗                        //
//    ██╔════╝██╔═══██╗████╗  ██║██╔══██╗██║╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝     ██╔╝                        //
//    ██║     ██║   ██║██╔██╗ ██║██║  ██║██║   ██║   ██║██║   ██║██╔██╗ ██║███████╗    ██╔╝                         //
//    ██║     ██║   ██║██║╚██╗██║██║  ██║██║   ██║   ██║██║   ██║██║╚██╗██║╚════██║   ██╔╝                          //
//    ╚██████╗╚██████╔╝██║ ╚████║██████╔╝██║   ██║   ██║╚██████╔╝██║ ╚████║███████║  ██╔╝                           //
//     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚═════╝ ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝  ╚═╝                            //
//                                                                                                                  //
//    ██████╗ ███████╗███████╗███████╗██████╗ ███████╗███╗   ██╗ ██████╗███████╗                                    //
//    ██╔══██╗██╔════╝██╔════╝██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝                                    //
//    ██████╔╝█████╗  █████╗  █████╗  ██████╔╝█████╗  ██╔██╗ ██║██║     █████╗                                      //
//    ██╔══██╗██╔══╝  ██╔══╝  ██╔══╝  ██╔══██╗██╔══╝  ██║╚██╗██║██║     ██╔══╝                                      //
//    ██║  ██║███████╗██║     ███████╗██║  ██║███████╗██║ ╚████║╚██████╗███████╗                                    //
//    ╚═╝  ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═══╝ ╚═════╝╚══════╝                                    //
//                                                                                                                  //
//    ███████╗██╗    ██╗██╗████████╗ ██████╗██╗  ██╗███████╗███████╗                                                //
//    ██╔════╝██║    ██║██║╚══██╔══╝██╔════╝██║  ██║██╔════╝██╔════╝                                                //
//    ███████╗██║ █╗ ██║██║   ██║   ██║     ███████║█████╗  ███████╗                                                //
//    ╚════██║██║███╗██║██║   ██║   ██║     ██╔══██║██╔══╝  ╚════██║                                                //
//    ███████║╚███╔███╔╝██║   ██║   ╚██████╗██║  ██║███████╗███████║                                                //
//    ╚══════╝ ╚══╝╚══╝ ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝                                                //
//                                                                                                                  //
//==================================================================================================================//
//                            RAMPER STOP CONDITIONS AND REFERENCE SWITCHES                                         //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Ramper Stop Condition and Reference Switch Parameters
/// @{
//--------------------------------------
//  Ramper Stop Conditions and Reference Switches
//--------------------------------------
/**
 * @brief Parameters for configuring ramper stop conditions and reference switch behavior.
 *
 * Table — Ramper Stop Conditions and Reference Switches:
 *  NUMBER | NAME                          | DESCRIPTION
 *  ------ | ----------------------------- | -----------------------------------------------------------------
 *   134   | STOP_ON_VELOCITY_DEVIATION    | Max velocity deviation before stop event [0...200000]. Default: 0. RW
 *   152   | STOP_ON_POSITION_DEVIATION    | Max position deviation before stop event [0...2147483647]. Default: 0. RWE
 *   154   | LATCH_POSITION                | Latched position at switch event [-2147483648...2147483647]. Default: 0. R
 *   161   | REFERENCE_SWITCH_ENABLE       | Bitwise enable for stopping on reference switch. See ReferenceSwitchEnable. Default: 0. RWE
 *   162   | REFERENCE_SWITCH_POLARITY_AND_SWAP | Bitwise config for switch polarity/swap. See ReferenceSwitchPolaritySwap. Default: 0. RWE
 *   163   | REFERENCE_SWITCH_LATCH_SETTINGS    | Bitwise config for latch behavior. See ReferenceSwitchLatchSettings. Default: 0. RWE
 *   164   | EVENT_STOP_SETTINGS           | Bitwise config for stop conditions. See EventStopSettings. Default: 0. RWE
 */
#define RAMPER_STOP_CONFIG_LIST(X) \
    X(STOP_ON_VELOCITY_DEVIATION,        134, /*!< Max velocity deviation before stop event [0...200000]. Default: 0. RW */) \
    X(STOP_ON_POSITION_DEVIATION,        152, /*!< Max position deviation before stop event [0...2147483647]. Default: 0. RWE */) \
    /* LATCH_POSITION handled in POSITION_CONTROL_LIST */ \
    X(REFERENCE_SWITCH_ENABLE,           161, /*!< Bitwise enable for stopping on reference switch. See ReferenceSwitchEnable. Default: 0. RWE */) \
    X(REFERENCE_SWITCH_POLARITY_AND_SWAP,162, /*!< Bitwise config for switch polarity/swap. See ReferenceSwitchPolaritySwap. Default: 0. RWE */) \
    X(REFERENCE_SWITCH_LATCH_SETTINGS,   163, /*!< Bitwise config for latch behavior. See ReferenceSwitchLatchSettings. Default: 0. RWE */) \
    X(EVENT_STOP_SETTINGS,               164, /*!< Bitwise config for stop conditions. See EventStopSettings. Default: 0. RWE */)

enum class RamperStopConfig : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    RAMPER_STOP_CONFIG_LIST(X)
    #undef X
};

inline const char* to_string(RamperStopConfig config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case RamperStopConfig::NAME: return #NAME;
        RAMPER_STOP_CONFIG_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Reference Switch Enable
/// @{
//--------------------------------------
//  Reference Switch Enable
//--------------------------------------
/**
 * @brief Bitwise enable for stopping when reference switch input is triggered.
 *
 * Table — Reference Switch Enable:
 *  NUMBER | NAME                  | DESCRIPTION
 *  ------ | --------------------- | -----------------------------------------------------------------
 *     0   | NO_STOP_ON_SWITCH_TRIGGERED | No stop on switch.
 *     1   | STOP_ON_L             | Stop on left switch.
 *     2   | STOP_ON_R             | Stop on right switch.
 *     3   | STOP_ON_R_AND_L       | Stop on right and left switches.
 *     4   | STOP_ON_H             | Stop on home switch.
 *     5   | STOP_ON_H_AND_L       | Stop on home and left switches.
 *     6   | STOP_ON_H_AND_R       | Stop on home and right switches.
 *     7   | STOP_ON_H_R_AND_L     | Stop on home, right, and left switches.
 */
#define REFERENCE_SWITCH_ENABLE_LIST(X) \
    X(NO_STOP_ON_SWITCH_TRIGGERED, 0, /*!< No stop on switch. */) \
    X(STOP_ON_L,                   1, /*!< Stop on left switch. */) \
    X(STOP_ON_R,                   2, /*!< Stop on right switch. */) \
    X(STOP_ON_R_AND_L,             3, /*!< Stop on right and left switches. */) \
    X(STOP_ON_H,                   4, /*!< Stop on home switch. */) \
    X(STOP_ON_H_AND_L,             5, /*!< Stop on home and left switches. */) \
    X(STOP_ON_H_AND_R,             6, /*!< Stop on home and right switches. */) \
    X(STOP_ON_H_R_AND_L,           7, /*!< Stop on home, right, and left switches. */)

enum class ReferenceSwitchEnable : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SWITCH_ENABLE_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSwitchEnable enable) {
    switch(enable) {
        #define X(NAME, VALUE, DOC) case ReferenceSwitchEnable::NAME: return #NAME;
        REFERENCE_SWITCH_ENABLE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REFERENCE_SWITCH_ENABLE_LIST
/// @}

/// @name Reference Switch Polarity and Swap
/// @{
//--------------------------------------
//  Reference Switch Polarity and Swap
//--------------------------------------
/**
 * @brief Bitwise configuration for reference switch polarity and swapping.
 *
 * Table — Reference Switch Polarity and Swap:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | -----------------------------------------------------------------
 *     0   | NOT_SWAPPED_NOT_INVERTED    | No swapping or inversion.
 *     1   | L_INVERTED                  | Left switch inverted.
 *     2   | R_INVERTED                  | Right switch inverted.
 *     3   | R_AND_L_INVERTED            | Right and left switches inverted.
 *     4   | H_INVERTED                  | Home switch inverted.
 *     5   | H_AND_L_INVERTED            | Home and left switches inverted.
 *     6   | H_AND_R_INVERTED            | Home and right switches inverted.
 *     7   | H_R_AND_L_INVERTED          | Home, right, and left switches inverted.
 *     8   | L_R_SWAPPED_L_INVERTED      | Left and right switches swapped, left inverted.
 *     9   | L_R_SWAPPED_R_INVERTED      | Left and right switches swapped, right inverted.
 *    10   | L_R_SWAPPED_R_AND_L_INVERTED| Left and right switches swapped, both inverted.
 *    11   | L_R_SWAPPED_H_INVERTED      | Left and right switches swapped, home inverted.
 *    12   | L_R_SWAPPED_H_AND_L_INVERTED| Left and right switches swapped, home and left inverted.
 *    13   | L_R_SWAPPED                 | Left and right switches swapped.
 *    14   | L_R_SWAPPED_H_AND_R_INVERTED| Left and right switches swapped, home and right inverted.
 *    15   | L_R_SWAPPED_H_R_AND_L_INVERTED | Left and right switches swapped, home, right, and left inverted.
 */
#define REFERENCE_SWITCH_POLARITY_SWAP_LIST(X) \
    X(NOT_SWAPPED_NOT_INVERTED,       0,  /*!< No swapping or inversion. */) \
    X(L_INVERTED,                     1,  /*!< Left switch inverted. */) \
    X(R_INVERTED,                     2,  /*!< Right switch inverted. */) \
    X(R_AND_L_INVERTED,               3,  /*!< Right and left switches inverted. */) \
    X(H_INVERTED,                     4,  /*!< Home switch inverted. */) \
    X(H_AND_L_INVERTED,               5,  /*!< Home and left switches inverted. */) \
    X(H_AND_R_INVERTED,               6,  /*!< Home and right switches inverted. */) \
    X(H_R_AND_L_INVERTED,             7,  /*!< Home, right, and left switches inverted. */) \
    X(L_R_SWAPPED_L_INVERTED,         8,  /*!< Left and right switches swapped, left inverted. */) \
    X(L_R_SWAPPED_R_INVERTED,         9,  /*!< Left and right switches swapped, right inverted. */) \
    X(L_R_SWAPPED_R_AND_L_INVERTED,  10,  /*!< Left and right switches swapped, both inverted. */) \
    X(L_R_SWAPPED_H_INVERTED,        11,  /*!< Left and right switches swapped, home inverted. */) \
    X(L_R_SWAPPED_H_AND_L_INVERTED,  12,  /*!< Left and right switches swapped, home and left inverted. */) \
    X(L_R_SWAPPED,                   13,  /*!< Left and right switches swapped. */) \
    X(L_R_SWAPPED_H_AND_R_INVERTED,  14,  /*!< Left and right switches swapped, home and right inverted. */) \
    X(L_R_SWAPPED_H_R_AND_L_INVERTED,15,  /*!< Left and right switches swapped, home, right, and left inverted. */)

enum class ReferenceSwitchPolaritySwap : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SWITCH_POLARITY_SWAP_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSwitchPolaritySwap config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case ReferenceSwitchPolaritySwap::NAME: return #NAME;
        REFERENCE_SWITCH_POLARITY_SWAP_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REFERENCE_SWITCH_POLARITY_SWAP_LIST
/// @}

/// @name Reference Switch Latch Settings
/// @{
//--------------------------------------
//  Reference Switch Latch Settings
//--------------------------------------
/**
 * @brief Bitwise configuration for reference switch latch behavior.
 *
 * Table — Reference Switch Latch Settings:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | -----------------------------------------------------------------
 *     0   | NO_TRIGGER                  | No latch triggered.
 *     1   | L_R_RISING_EDGE             | Latch on rising edge of left/right switch.
 *     2   | L_R_FALLING_EDGE            | Latch on falling edge of left/right switch.
 *     3   | L_R_BOTH_EDGES              | Latch on both edges of left/right switch.
 *     4   | H_RISING_EDGE               | Latch on rising edge of home switch.
 *     5   | H_L_R_RISING_EDGE           | Latch on rising edge of home and left/right switches.
 *     6   | H_RISING_L_R_FALLING_EDGE   | Latch on rising edge of home and falling edge of left/right switches.
 *     7   | H_RISING_L_R_BOTH_EDGES     | Latch on rising edge of home and both edges of left/right switches.
 *     8   | H_FALLING_EDGE              | Latch on falling edge of home switch.
 *     9   | H_FALLING_L_R_RISING_EDGE   | Latch on falling edge of home and rising edge of left/right switches.
 *    10   | H_L_R_FALLING_EDGE          | Latch on falling edge of home and left/right switches.
 *    11   | H_FALLING_L_R_BOTH_EDGES    | Latch on falling edge of home and both edges of left/right switches.
 *    12   | H_BOTH_EDGES                | Latch on both edges of home switch.
 *    13   | H_BOTH_L_R_RISING_EDGE      | Latch on both edges of home and rising edge of left/right switches.
 *    14   | H_BOTH_L_R_FALLING_EDGE     | Latch on both edges of home and falling edge of left/right switches.
 *    15   | H_L_R_BOTH_EDGES            | Latch on both edges of home and left/right switches.
 */
#define REFERENCE_SWITCH_LATCH_SETTINGS_LIST(X) \
    X(NO_TRIGGER,                  0,  /*!< No latch triggered. */) \
    X(L_R_RISING_EDGE,             1,  /*!< Latch on rising edge of left/right switch. */) \
    X(L_R_FALLING_EDGE,            2,  /*!< Latch on falling edge of left/right switch. */) \
    X(L_R_BOTH_EDGES,              3,  /*!< Latch on both edges of left/right switch. */) \
    X(H_RISING_EDGE,               4,  /*!< Latch on rising edge of home switch. */) \
    X(H_L_R_RISING_EDGE,           5,  /*!< Latch on rising edge of home and left/right switches. */) \
    X(H_RISING_L_R_FALLING_EDGE,   6,  /*!< Latch on rising edge of home and falling edge of left/right switches. */) \
    X(H_RISING_L_R_BOTH_EDGES,     7,  /*!< Latch on rising edge of home and both edges of left/right switches. */) \
    X(H_FALLING_EDGE,              8,  /*!< Latch on falling edge of home switch. */) \
    X(H_FALLING_L_R_RISING_EDGE,   9,  /*!< Latch on falling edge of home and rising edge of left/right switches. */) \
    X(H_L_R_FALLING_EDGE,          10, /*!< Latch on falling edge of home and left/right switches. */) \
    X(H_FALLING_L_R_BOTH_EDGES,    11, /*!< Latch on falling edge of home and both edges of left/right switches. */) \
    X(H_BOTH_EDGES,                12, /*!< Latch on both edges of home switch. */) \
    X(H_BOTH_L_R_RISING_EDGE,      13, /*!< Latch on both edges of home and rising edge of left/right switches. */) \
    X(H_BOTH_L_R_FALLING_EDGE,     14, /*!< Latch on both edges of home and falling edge of left/right switches. */) \
    X(H_L_R_BOTH_EDGES,            15, /*!< Latch on both edges of home and left/right switches. */)

enum class ReferenceSwitchLatchSettings : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SWITCH_LATCH_SETTINGS_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSwitchLatchSettings setting) {
    switch(setting) {
        #define X(NAME, VALUE, DOC) case ReferenceSwitchLatchSettings::NAME: return #NAME;
        REFERENCE_SWITCH_LATCH_SETTINGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REFERENCE_SWITCH_LATCH_SETTINGS_LIST
/// @}

/// @name Event Stop Settings
/// @{
//--------------------------------------
//  Event Stop Settings
//--------------------------------------
/**
 * @brief Bitwise configuration for stop conditions.
 *
 * Table — Event Stop Settings:
 *  NUMBER | NAME                              | DESCRIPTION
 *  ------ | --------------------------------- | -----------------------------------------------------------------
 *     0   | DO_HARD_STOP                      | Hard stop on event.
 *     1   | DO_SOFT_STOP                      | Soft stop (ramp down) on event.
 *     2   | STOP_ON_POS_DEVIATION             | Stop on position deviation.
 *     3   | STOP_ON_POS_DEVIATION_SOFT_STOP   | Stop on position deviation, soft stop.
 *     4   | STOP_ON_VEL_DEVIATION             | Stop on velocity deviation.
 *     5   | STOP_ON_VEL_DEVIATION_SOFT_STOP   | Stop on velocity deviation, soft stop.
 *     6   | STOP_ON_POS_VEL_DEVIATION         | Stop on position or velocity deviation.
 *     7   | STOP_ON_POS_VEL_DEVIATION_SOFT_STOP | Stop on position or velocity deviation, soft stop.
 */
#define EVENT_STOP_SETTINGS_LIST(X) \
    X(DO_HARD_STOP,                      0, /*!< Hard stop on event. */) \
    X(DO_SOFT_STOP,                      1, /*!< Soft stop (ramp down) on event. */) \
    X(STOP_ON_POS_DEVIATION,             2, /*!< Stop on position deviation. */) \
    X(STOP_ON_POS_DEVIATION_SOFT_STOP,   3, /*!< Stop on position deviation, soft stop. */) \
    X(STOP_ON_VEL_DEVIATION,             4, /*!< Stop on velocity deviation. */) \
    X(STOP_ON_VEL_DEVIATION_SOFT_STOP,   5, /*!< Stop on velocity deviation, soft stop. */) \
    X(STOP_ON_POS_VEL_DEVIATION,         6, /*!< Stop on position or velocity deviation. */) \
    X(STOP_ON_POS_VEL_DEVIATION_SOFT_STOP, 7, /*!< Stop on position or velocity deviation, soft stop. */)

enum class EventStopSettings : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    EVENT_STOP_SETTINGS_LIST(X)
    #undef X
};

inline const char* to_string(EventStopSettings setting) {
    switch(setting) {
        #define X(NAME, VALUE, DOC) case EventStopSettings::NAME: return #NAME;
        EVENT_STOP_SETTINGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef EVENT_STOP_SETTINGS_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗ ██╗ ██████╗ ██╗   ██╗ █████╗ ██████╗     ███████╗██╗██╗  ████████╗███████╗██████╗                     //
//    ██╔══██╗██║██╔═══██╗██║   ██║██╔══██╗██╔══██╗    ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗                    //
//    ██████╔╝██║██║   ██║██║   ██║███████║██║  ██║    █████╗  ██║██║     ██║   █████╗  ██████╔╝                    //
//    ██╔══██╗██║██║▄▄ ██║██║   ██║██╔══██║██║  ██║    ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗                    //
//    ██████╔╝██║╚██████╔╝╚██████╔╝██║  ██║██████╔╝    ██║     ██║███████╗██║   ███████╗██║  ██║                    //
//    ╚═════╝ ╚═╝ ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝╚═════╝     ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝                    //
//                                                                                                                  //
//==================================================================================================================//
//                                            BIQUAD FILTER SETUP SECTION                                           //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Biquad Filter Parameters
/// @{
//--------------------------------------
//  Biquad Filter Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring biquad filters for torque and velocity control loops.
 *
 * The TMC9660 Parameter Mode includes biquad filters to enhance the performance of its velocity and torque control loops.
 * The biquad filter is a second-order digital filter, operating on current and past input/output samples using configurable coefficients.
 * 
 * The filter output Y(n) is calculated as:
 *   Y(n) = X(n)*b0 + X(n-1)*b1 + X(n-2)*b2 + Y(n-1)*a1 + Y(n-2)*a2
 * where:
 *   - X(n): current input sample
 *   - X(n-1), X(n-2): previous input samples
 *   - Y(n-1), Y(n-2): previous output samples
 *   - a1, a2, b0, b1, b2: filter coefficients (Q4.20 format, 24-bit)
 *
 * The velocity biquad filter (enabled by default) filters the actual velocity used as input for the velocity controller.
 * The torque biquad filter can be enabled to filter the target torque value.
 *
 * NUMBER | NAME                             | DESCRIPTION
 * ------ | -------------------------------- | -----------------------------------------------
 * 318    | TARGET_TORQUE_BIQUAD_FILTER_ENABLE      | Enable target torque biquad filter. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 * 319    | TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1    | Target torque biquad filter aCoeff_1 [-2147483648, 2147483647]. Default: 0. RWE
 * 320    | TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2    | Target torque biquad filter aCoeff_2 [-2147483648, 2147483647]. Default: 0. RWE
 * 321    | TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0    | Target torque biquad filter bCoeff_0 [-2147483648, 2147483647]. Default: 1048576. RWE
 * 322    | TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1    | Target torque biquad filter bCoeff_1 [-2147483648, 2147483647]. Default: 0. RWE
 * 323    | TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2    | Target torque biquad filter bCoeff_2 [-2147483648, 2147483647]. Default: 0. RWE
 * 324    | ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE    | Enable actual velocity biquad filter. 0: DISABLED, 1: ENABLED. Default: 1. RWE
 * 325    | ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1  | Actual velocity biquad filter aCoeff_1 [-2147483648, 2147483647]. Default: 1849195. RWE
 * 326    | ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2  | Actual velocity biquad filter aCoeff_2 [-2147483648, 2147483647]. Default: 15961938. RWE
 * 327    | ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0  | Actual velocity biquad filter bCoeff_0 [-2147483648, 2147483647]. Default: 3665. RWE
 * 328    | ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1  | Actual velocity biquad filter bCoeff_1 [-2147483648, 2147483647]. Default: 7329. RWE
 * 329    | ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2  | Actual velocity biquad filter bCoeff_2 [-2147483648, 2147483647]. Default: 3665. RWE
 */
#define BIQUAD_FILTER_LIST(X) \
    X(TARGET_TORQUE_BIQUAD_FILTER_ENABLE,     318, /*!< Enable target torque biquad filter. 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1,   319, /*!< Target torque biquad filter aCoeff_1 [-2147483648, 2147483647]. Default: 0. RWE */) \
    X(TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2,   320, /*!< Target torque biquad filter aCoeff_2 [-2147483648, 2147483647]. Default: 0. RWE */) \
    X(TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0,   321, /*!< Target torque biquad filter bCoeff_0 [-2147483648, 2147483647]. Default: 1048576. RWE */) \
    X(TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1,   322, /*!< Target torque biquad filter bCoeff_1 [-2147483648, 2147483647]. Default: 0. RWE */) \
    X(TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2,   323, /*!< Target torque biquad filter bCoeff_2 [-2147483648, 2147483647]. Default: 0. RWE */) \
    X(ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE,   324, /*!< Enable actual velocity biquad filter. 0: DISABLED, 1: ENABLED. Default: 1. RWE */) \
    X(ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1, 325, /*!< Actual velocity biquad filter aCoeff_1 [-2147483648, 2147483647]. Default: 1849195. RWE */) \
    X(ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2, 326, /*!< Actual velocity biquad filter aCoeff_2 [-2147483648, 2147483647]. Default: 15961938. RWE */) \
    X(ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0, 327, /*!< Actual velocity biquad filter bCoeff_0 [-2147483648, 2147483647]. Default: 3665. RWE */) \
    X(ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1, 328, /*!< Actual velocity biquad filter bCoeff_1 [-2147483648, 2147483647]. Default: 7329. RWE */) \
    X(ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2, 329,  /*!< Actual velocity biquad filter bCoeff_2 [-2147483648, 2147483647]. Default: 3665. RWE */)

enum class BiquadFilter : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    BIQUAD_FILTER_LIST(X)
    #undef X
};

inline const char* to_string(BiquadFilter e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case BiquadFilter::NAME: return #NAME;
        BIQUAD_FILTER_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Biquad Filter Enable/Disable
/// @{
//--------------------------------------
//  Biquad Filter Enable/Disable
//--------------------------------------
/**
 * @brief Enumerates enable/disable options for biquad filters.
 *
 * Table — Biquad Filter Enable/Disable:
 *  NUMBER | NAME     | DESCRIPTION
 *  ------ | -------- | -------------------------------
 *     0   | DISABLED | Filter disabled.
 *     1   | ENABLED  | Filter enabled.
 */
#define BIQUAD_FILTER_ENABLE_LIST(X) \
    X(DISABLED, 0, /*!< Filter disabled. */) \
    X(ENABLED,  1, /*!< Filter enabled. */)

enum class BiquadFilterEnable : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    BIQUAD_FILTER_ENABLE_LIST(X)
    #undef X
};

inline const char* to_string(BiquadFilterEnable enable) {
    switch(enable) {
        #define X(NAME, VALUE, DOC) case BiquadFilterEnable::NAME: return #NAME;
        BIQUAD_FILTER_ENABLE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef BIQUAD_FILTER_ENABLE_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗ █████╗ ██╗   ██╗██╗  ████████╗    ██╗  ██╗ █████╗ ███╗   ██╗██████╗ ██╗     ██╗███╗   ██╗ ██████╗    //
//    ██╔════╝██╔══██╗██║   ██║██║  ╚══██╔══╝    ██║  ██║██╔══██╗████╗  ██║██╔══██╗██║     ██║████╗  ██║██╔════╝    //
//    █████╗  ███████║██║   ██║██║     ██║       ███████║███████║██╔██╗ ██║██║  ██║██║     ██║██╔██╗ ██║██║  ███╗   //
//    ██╔══╝  ██╔══██║██║   ██║██║     ██║       ██╔══██║██╔══██║██║╚██╗██║██║  ██║██║     ██║██║╚██╗██║██║   ██║   //
//    ██║     ██║  ██║╚██████╔╝███████╗██║       ██║  ██║██║  ██║██║ ╚████║██████╔╝███████╗██║██║ ╚████║╚██████╔╝   //
//    ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝       ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝ ╚══════╝╚═╝╚═╝  ╚═══╝ ╚═════╝    //
//                                                                                                                  //
//==================================================================================================================//
//                                               FAULT HANDLING SECTION                                             //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Fault Handling Parameters
/// @{
//--------------------------------------
//  Fault Handling Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring system behavior on fault conditions.
 *
 * The TMC9660 supports advanced fault handling for overtemperature, I²T, and gate driver faults.
 * The system can be configured to react with open-circuit, electrical braking, and/or mechanical braking.
 * For gate driver faults, a retry mechanism is available. The number of retries and retry behavior are configurable.
 * If all retries fail, the standard drive fault behavior is applied.
 *
 * Table — Fault Handling Parameters:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | -----------------------------------------------------------------
 *   286   | GDRV_RETRY_BEHAVIOUR        | State after a gate driver fault. See GdrvRetryBehaviour. Default: 0 (OPEN_CIRCUIT). RWE
 *   287   | DRIVE_FAULT_BEHAVIOUR       | State after all retries fail. See DriveFaultBehaviour. Default: 0 (OPEN_CIRCUIT). RWE
 *   288   | FAULT_HANDLER_NUMBER_OF_RETRIES | Max number of retries per detected fault [0...255]. Default: 5. RWE
 */
#define FAULT_HANDLING_LIST(X) \
    X(GDRV_RETRY_BEHAVIOUR, 286,         /*!< State after a gate driver fault. See GdrvRetryBehaviour. Default: 0 (OPEN_CIRCUIT). RWE */) \
    X(DRIVE_FAULT_BEHAVIOUR, 287,        /*!< State after all retries fail. See DriveFaultBehaviour. Default: 0 (OPEN_CIRCUIT). RWE */) \
    X(FAULT_HANDLER_NUMBER_OF_RETRIES, 288, /*!< Max number of retries per detected fault [0...255]. Default: 5. RWE */)

enum class FaultHandling : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    FAULT_HANDLING_LIST(X)
    #undef X
};

inline const char* to_string(FaultHandling config) {
    switch(config) {
        #define X(NAME, VALUE, DOC) case FaultHandling::NAME: return #NAME;
        FAULT_HANDLING_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Gate Driver Retry Behaviour
/// @{
//--------------------------------------
//  Gate Driver Retry Behaviour
//--------------------------------------
/**
 * @brief System state after a gate driver fault occurs.
 *
 * Table — Gate Driver Retry Behaviour:
 *  NUMBER | NAME            | DESCRIPTION
 *  ------ | --------------- | -----------------------------------------------------------------
 *     0   | OPEN_CIRCUIT    | Switch off and discharge gates; motor spins freely.
 *     1   | ELECTRICAL_BRAKING | Switch off and, if possible, enable LS or HS gates for electrical braking.
 */
#define GDRV_RETRY_BEHAVIOUR_LIST(X) \
    X(OPEN_CIRCUIT, 0,        /*!< Switch off and discharge gates; motor spins freely. */) \
    X(ELECTRICAL_BRAKING, 1,  /*!< Switch off and, if possible, enable LS or HS gates for electrical braking. */)

enum class GdrvRetryBehaviour : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GDRV_RETRY_BEHAVIOUR_LIST(X)
    #undef X
};

inline const char* to_string(GdrvRetryBehaviour behaviour) {
    switch(behaviour) {
        #define X(NAME, VALUE, DOC) case GdrvRetryBehaviour::NAME: return #NAME;
        GDRV_RETRY_BEHAVIOUR_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GDRV_RETRY_BEHAVIOUR_LIST
/// @}

/// @name Drive Fault Behaviour
/// @{
//--------------------------------------
//  Drive Fault Behaviour
//--------------------------------------
/**
 * @brief System state after all retries fail following a fault.
 *
 * Table — Drive Fault Behaviour:
 *  NUMBER | NAME                                | DESCRIPTION
 *  ------ | ----------------------------------- | -----------------------------------------------------------------
 *     0   | OPEN_CIRCUIT                       | Switch off and discharge LS/HS gates; motor spins freely.
 *     1   | ELECTRICAL_BRAKING                 | Switch off and, if possible, enable LS/HS gates for electrical braking.
 *     2   | MECHANICAL_BRAKING_AND_OPEN_CIRCUIT| Switch off, discharge LS/HS gates, and engage mechanical brake if configured.
 *     3   | MECHANICAL_AND_ELECTRICAL_BRAKING  | Switch off, enable LS/HS gates if possible, and engage mechanical brake if configured.
 */
#define DRIVE_FAULT_BEHAVIOUR_LIST(X) \
    X(OPEN_CIRCUIT,                       0, /*!< Switch off and discharge LS/HS gates; motor spins freely. */) \
    X(ELECTRICAL_BRAKING,                 1, /*!< Switch off and, if possible, enable LS/HS gates for electrical braking. */) \
    X(MECHANICAL_BRAKING_AND_OPEN_CIRCUIT,2, /*!< Switch off, discharge LS/HS gates, and engage mechanical brake if configured. */) \
    X(MECHANICAL_AND_ELECTRICAL_BRAKING,  3, /*!< Switch off, enable LS/HS gates if possible, and engage mechanical brake if configured. */)

enum class DriveFaultBehaviour : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    DRIVE_FAULT_BEHAVIOUR_LIST(X)
    #undef X
};

inline const char* to_string(DriveFaultBehaviour behaviour) {
    switch(behaviour) {
        #define X(NAME, VALUE, DOC) case DriveFaultBehaviour::NAME: return #NAME;
        DRIVE_FAULT_BEHAVIOUR_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef DRIVE_FAULT_BEHAVIOUR_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗██╗████████╗    ███╗   ███╗ ██████╗ ███╗   ██╗██╗████████╗ ██████╗ ██████╗                                 //
//    ██║██║╚══██╔══╝    ████╗ ████║██╔═══██╗████╗  ██║██║╚══██╔══╝██╔═══██╗██╔══██╗                                //
//    ██║██║   ██║       ██╔████╔██║██║   ██║██╔██╗ ██║██║   ██║   ██║   ██║██████╔╝                                //
//    ██║██║   ██║       ██║╚██╔╝██║██║   ██║██║╚██╗██║██║   ██║   ██║   ██║██╔══██╗                                //
//    ██║██║   ██║       ██║ ╚═╝ ██║╚██████╔╝██║ ╚████║██║   ██║   ╚██████╔╝██║  ██║                                //
//    ╚═╝╚═╝   ╚═╝       ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝                                //
//                                                                                                                  //
//==================================================================================================================//
//                                               IIT MONITOR SECTION                                                //
//==================================================================================================================//

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name IIT Monitor Parameters
/// @{
//--------------------------------------
//  IIT Monitor Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring IIT (I²t) monitoring and protection.
 *
 * The IIT monitor protects against overtemperature in sustained load conditions by monitoring the total current
 * flowing into the motor (torque and flux). Two independent IIT monitoring windows are supported, each with
 * its own time constant and limit. If the squared current sum exceeds the limit, an emergency stop is triggered.
 * The behavior is defined by DRIVE_FAULT_BEHAVIOUR. Status flags indicate active protection and limit exceeded.
 *
 * - The total motor current is available via ACTUAL_TOTAL_MOTOR_CURRENT.
 * - Each window accumulates squared current over a time set by THERMAL_WINDING_TIME_CONSTANT_1/2.
 * - The sum for each window is available via IIT_SUM_1/2.
 * - Limits are set by IIT_LIMIT_1/2. If not set to max, protection is active.
 * - If the sum exceeds the limit, the corresponding IIT_EXCEEDED flag is set and the motor is stopped.
 * - Both sums can be reset with RESET_IIT_SUMS.
 * - The update rate depends on MOTOR_PWM_FREQUENCY.
 *
 * Table — IIT Monitor Parameters:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | -----------------------------------------------------------------
 *   224   | THERMAL_WINDING_TIME_CONSTANT_1 | Time constant for IIT window 1 [ms]. Default: 3000.
 *   225   | IIT_LIMIT_1                 | IIT limit for window 1 [A^2 x ms]. Default: 4294967295.
 *   226   | IIT_SUM_1                   | Actual IIT sum for window 1 [A^2 x ms]. Default: 0.
 *   227   | THERMAL_WINDING_TIME_CONSTANT_2 | Time constant for IIT window 2 [ms]. Default: 6000.
 *   228   | IIT_LIMIT_2                 | IIT limit for window 2 [A^2 x ms]. Default: 4294967295.
 *   229   | IIT_SUM_2                   | Actual IIT sum for window 2 [A^2 x ms]. Default: 0.
 *   230   | RESET_IIT_SUMS              | Write to reset both IIT sums. Default: 0.
 *   231   | ACTUAL_TOTAL_MOTOR_CURRENT  | Total current through motor windings [mA]. Default: 0.
 */
#define IIT_MONITOR_LIST(X) \
    X(THERMAL_WINDING_TIME_CONSTANT_1, 224, /*!< Time constant for IIT window 1 [ms]. Default: 3000. */) \
    X(IIT_LIMIT_1,                     225, /*!< IIT limit for window 1 [A^2 x ms]. Default: 4294967295. */) \
    X(IIT_SUM_1,                       226, /*!< Actual IIT sum for window 1 [A^2 x ms]. Default: 0. */) \
    X(THERMAL_WINDING_TIME_CONSTANT_2, 227, /*!< Time constant for IIT window 2 [ms]. Default: 6000. */) \
    X(IIT_LIMIT_2,                     228, /*!< IIT limit for window 2 [A^2 x ms]. Default: 4294967295. */) \
    X(IIT_SUM_2,                       229, /*!< Actual IIT sum for window 2 [A^2 x ms]. Default: 0. */) \
    X(RESET_IIT_SUMS,                  230, /*!< Write to reset both IIT sums. Default: 0. */) \
    X(ACTUAL_TOTAL_MOTOR_CURRENT,      231, /*!< Total current through motor windings [mA]. Default: 0. */)

enum class IitMonitor : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    IIT_MONITOR_LIST(X)
    #undef X
};

inline const char* to_string(IitMonitor e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case IitMonitor::NAME: return #NAME;
        IIT_MONITOR_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ████████╗███████╗███╗   ███╗██████╗ ███████╗██████╗  █████╗ ████████╗██╗   ██╗██████╗ ███████╗                //
//    ╚══██╔══╝██╔════╝████╗ ████║██╔══██╗██╔════╝██╔══██╗██╔══██╗╚══██╔══╝██║   ██║██╔══██╗██╔════╝                //
//       ██║   █████╗  ██╔████╔██║██████╔╝█████╗  ██████╔╝███████║   ██║   ██║   ██║██████╔╝█████╗                  //
//       ██║   ██╔══╝  ██║╚██╔╝██║██╔═══╝ ██╔══╝  ██╔══██╗██╔══██║   ██║   ██║   ██║██╔══██╗██╔══╝                  //
//       ██║   ███████╗██║ ╚═╝ ██║██║     ███████╗██║  ██║██║  ██║   ██║   ╚██████╔╝██║  ██║███████╗                //
//       ╚═╝   ╚══════╝╚═╝     ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚══════╝                //
//                                                                                                                  //
//    ██████╗ ██████╗  ██████╗ ████████╗███████╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗                     //
//    ██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝                     //
//    ██████╔╝██████╔╝██║   ██║   ██║   █████╗  ██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗                     //
//    ██╔═══╝ ██╔══██╗██║   ██║   ██║   ██╔══╝  ██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║                     //
//    ██║     ██║  ██║╚██████╔╝   ██║   ███████╗╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║                     //
//    ╚═╝     ╚═╝  ╚═╝ ╚═════╝    ╚═╝   ╚══════╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝                     //
//                                                                                                                  //
//==================================================================================================================//
//                                   TEMPERATURE PROTECTIONS SECTION                                                //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Temperature Protection Parameters
/// @{
//--------------------------------------
//  Temperature Protection Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring temperature protection using external and internal sensors.
 *
 * The TMC9660 integrates protections using both an external analog temperature sensor (AIN3) and an internal chip sensor.
 * - EXTERNAL_TEMPERATURE can be converted to voltage: Voltage[V] = par(EXTERNAL_TEMPERATURE) × 2.5V / (2^16 - 1)
 * - CHIP_TEMPERATURE can be converted to °C: Temperature[°C] = par(CHIP_TEMPERATURE) × 0.01615 - 268.15
 * - Warning and shutdown thresholds can be set for both sensors. Exceeding a warning threshold sets a warning flag.
 *   Exceeding a shutdown threshold initiates a motor shutdown, as defined by DRIVE_FAULT_BEHAVIOUR.
 * - To restart after shutdown, clear the corresponding error flag.
 *
 * Table — Temperature Protection Parameters:
 *  NUMBER | NAME                                  | DESCRIPTION
 *  ------ | ------------------------------------- | -----------------------------------------------------------------
 *   293   | EXTERNAL_TEMPERATURE                  | External temperature sensor value [0...65535].
 *   294   | EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD | Shutdown threshold for external temperature [0...65535]. Default: 65535.
 *   295   | EXTERNAL_TEMPERATURE_WARNING_THRESHOLD  | Warning threshold for external temperature [0...65535]. Default: 65535.
 *   296   | CHIP_TEMPERATURE                      | Internal chip temperature value [0...65535].
 *   297   | CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD   | Shutdown threshold for chip temperature [0...65535]. Default: 65535.
 *   298   | CHIP_TEMPERATURE_WARNING_THRESHOLD    | Warning threshold for chip temperature [0...65535]. Default: 65535.
 */
#define TEMPERATURE_PROTECTION_LIST(X) \
    X(EXTERNAL_TEMPERATURE,                    293, /*!< External temperature sensor value [0...65535]. */) \
    X(EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD, 294, /*!< Shutdown threshold for external temperature [0...65535]. Default: 65535. */) \
    X(EXTERNAL_TEMPERATURE_WARNING_THRESHOLD,  295, /*!< Warning threshold for external temperature [0...65535]. Default: 65535. */) \
    X(CHIP_TEMPERATURE,                        296, /*!< Internal chip temperature value [0...65535]. */) \
    X(CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD,     297, /*!< Shutdown threshold for chip temperature [0...65535]. Default: 65535. */) \
    X(CHIP_TEMPERATURE_WARNING_THRESHOLD,      298, /*!< Warning threshold for chip temperature [0...65535]. Default: 65535. */)

enum class TemperatureProtection : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    TEMPERATURE_PROTECTION_LIST(X)
    #undef X
};

inline const char* to_string(TemperatureProtection e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case TemperatureProtection::NAME: return #NAME;
        TEMPERATURE_PROTECTION_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗  ██╗███████╗ █████╗ ██████╗ ████████╗██████╗ ███████╗ █████╗ ████████╗                                    //
//    ██║  ██║██╔════╝██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗██╔════╝██╔══██╗╚══██╔══╝                                    //
//    ███████║█████╗  ███████║██████╔╝   ██║   ██████╔╝█████╗  ███████║   ██║                                       //
//    ██╔══██║██╔══╝  ██╔══██║██╔══██╗   ██║   ██╔══██╗██╔══╝  ██╔══██║   ██║                                       //
//    ██║  ██║███████╗██║  ██║██║  ██║   ██║   ██████╔╝███████╗██║  ██║   ██║                                       //
//    ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝                                       //
//                                                                                                                  //
//    ███╗   ███╗ ██████╗ ███╗   ██╗██╗████████╗ ██████╗ ██████╗ ██╗███╗   ██╗ ██████╗                              //
//    ████╗ ████║██╔═══██╗████╗  ██║██║╚══██╔══╝██╔═══██╗██╔══██╗██║████╗  ██║██╔════╝                              //
//    ██╔████╔██║██║   ██║██╔██╗ ██║██║   ██║   ██║   ██║██████╔╝██║██╔██╗ ██║██║  ███╗                             //
//    ██║╚██╔╝██║██║   ██║██║╚██╗██║██║   ██║   ██║   ██║██╔══██╗██║██║╚██╗██║██║   ██║                             //
//    ██║ ╚═╝ ██║╚██████╔╝██║ ╚████║██║   ██║   ╚██████╔╝██║  ██║██║██║ ╚████║╚██████╔╝                             //
//    ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝ ╚═════╝                              //
//                                                                                                                  //
//==================================================================================================================//
//                                   HEARTBEAT MONITORING SECTION                                                   //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Heartbeat Monitoring Parameters
/// @{
//--------------------------------------
//  Heartbeat Monitoring Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring heartbeat monitoring.
 *
 * @warning These are **global bank-0** parameters (SGP/GGP), not axis parameters, and are not
 * part of ::Parameters. Prefer GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG / _TIMEOUT, which
 * the driver APIs bind to bank 0 automatically.
 *
 * The heartbeat monitor checks for regular communication on UART and/or SPI. If no datagram is received within
 * the configured timeout, the system initiates a motor shutdown (behavior defined by DRIVE_FAULT_BEHAVIOUR).
 * The status flag HEARTBEAT_STOPPED is raised.
 *
 * Table — Heartbeat Monitoring Parameters:
 *  NUMBER | NAME                      | DESCRIPTION
 *  ------ | ------------------------- | -----------------------------------------------
 *     3   | HEARTBEAT_MONITORING_CONFIG | Heartbeat monitoring config. 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI.
 *     4   | HEARTBEAT_MONITORING_TIMEOUT | Heartbeat timeout in ms [1...4294967295]. Default: 100.
 */
#define HEARTBEAT_MONITORING_LIST(X) \
    X(HEARTBEAT_MONITORING_CONFIG,  3, /*!< Heartbeat monitoring config. 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. */) \
    X(HEARTBEAT_MONITORING_TIMEOUT, 4, /*!< Heartbeat timeout in ms [1...4294967295]. Default: 100. */)

enum class HeartbeatMonitoring : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    HEARTBEAT_MONITORING_LIST(X)
    #undef X
};

inline const char* to_string(HeartbeatMonitoring e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case HeartbeatMonitoring::NAME: return #NAME;
        HEARTBEAT_MONITORING_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Heartbeat Monitoring Config
/// @{
//--------------------------------------
//  Heartbeat Monitoring Config
//--------------------------------------
/**
 * @brief Enumerates heartbeat monitoring interface selection.
 *
 * Table — Heartbeat Monitoring Config:
 *  NUMBER | NAME                      | DESCRIPTION
 *  ------ | ------------------------- | -----------------------------------------------
 *     0   | DISABLED                  | Heartbeat monitoring disabled.
 *     1   | TMCL_UART_INTERFACE       | Monitor TMCL UART interface.
 *     2   | SPI_INTERFACE             | Monitor SPI interface.
 *     3   | TMCL_UART_AND_SPI_INTERFACE | Monitor both UART and SPI.
 */
#define HEARTBEAT_MONITORING_CONFIG_LIST(X) \
    X(DISABLED,                    0, /*!< Heartbeat monitoring disabled. */) \
    X(TMCL_UART_INTERFACE,         1, /*!< Monitor TMCL UART interface. */) \
    X(SPI_INTERFACE,               2, /*!< Monitor SPI interface. */) \
    X(TMCL_UART_AND_SPI_INTERFACE, 3, /*!< Monitor both UART and SPI. */)

enum class HeartbeatMonitoringConfig : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    HEARTBEAT_MONITORING_CONFIG_LIST(X)
    #undef X
};

inline const char* to_string(HeartbeatMonitoringConfig e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case HeartbeatMonitoringConfig::NAME: return #NAME;
        HEARTBEAT_MONITORING_CONFIG_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef HEARTBEAT_MONITORING_CONFIG_LIST
/// @}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗ ██████╗  █████╗ ██╗  ██╗███████╗     ██████╗██╗  ██╗ ██████╗ ██████╗ ██████╗ ███████╗██████╗          //
//    ██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝██╔════╝    ██╔════╝██║  ██║██╔═══██╗██╔══██╗██╔══██╗██╔════╝██╔══██╗         //
//    ██████╔╝██████╔╝███████║█████╔╝ █████╗      ██║     ███████║██║   ██║██████╔╝██████╔╝█████╗  ██████╔╝         //
//    ██╔══██╗██╔══██╗██╔══██║██╔═██╗ ██╔══╝      ██║     ██╔══██║██║   ██║██╔═══╝ ██╔═══╝ ██╔══╝  ██╔══██╗         //
//    ██████╔╝██║  ██║██║  ██║██║  ██╗███████╗    ╚██████╗██║  ██║╚██████╔╝██║     ██║     ███████╗██║  ██║         //
//    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝     ╚═════╝╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝     ╚══════╝╚═╝  ╚═╝         //
//                                                                                                                  //
//==================================================================================================================//
//                                               BRAKE CHOPPER SECTION                                              //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Brake Chopper Parameters
/// @{
//--------------------------------------
//  Brake Chopper Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring the brake chopper functionality.
 *
 * The brake chopper dissipates excess energy via an external brake resistor and MOSFET when supply voltage exceeds a set limit.
 * - Enable with BRAKE_CHOPPER_ENABLE.
 * - When supply voltage exceeds BRAKE_CHOPPER_VOLTAGE_LIMIT, the brake chopper MOSFET is activated.
 * - The MOSFET is deactivated when voltage drops below (limit - hysteresis).
 *
 * Table — Brake Chopper Parameters:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | -----------------------------------------------------------------
 *   212   | BRAKE_CHOPPER_ENABLE        | Enable brake chopper. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 *   213   | BRAKE_CHOPPER_VOLTAGE_LIMIT | Voltage limit [0.1V] to activate brake chopper. 50...1000. Default: 260. RWE
 *   214   | BRAKE_CHOPPER_HYSTERESIS    | Hysteresis [0.1V] for deactivation. 0...50. Default: 5. RWE
 */
#define BRAKE_CHOPPER_LIST(X) \
    X(BRAKE_CHOPPER_ENABLE,        212, /*!< Enable brake chopper. 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(BRAKE_CHOPPER_VOLTAGE_LIMIT, 213, /*!< Voltage limit [0.1V] to activate brake chopper. 50...1000. Default: 260. RWE */) \
    X(BRAKE_CHOPPER_HYSTERESIS,    214, /*!< Hysteresis [0.1V] for deactivation. 0...50. Default: 5. RWE */)

enum class BrakeChopper : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    BRAKE_CHOPPER_LIST(X)
    #undef X
};

inline const char* to_string(BrakeChopper e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case BrakeChopper::NAME: return #NAME;
        BRAKE_CHOPPER_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███╗   ███╗███████╗ ██████╗██╗  ██╗ █████╗ ███╗   ██╗██╗ ██████╗ █████╗ ██╗                                   //
//    ████╗ ████║██╔════╝██╔════╝██║  ██║██╔══██╗████╗  ██║██║██╔════╝██╔══██╗██║                                   //
//    ██╔████╔██║█████╗  ██║     ███████║███████║██╔██╗ ██║██║██║     ███████║██║                                   //
//    ██║╚██╔╝██║██╔══╝  ██║     ██╔══██║██╔══██║██║╚██╗██║██║██║     ██╔══██║██║                                   //
//    ██║ ╚═╝ ██║███████╗╚██████╗██║  ██║██║  ██║██║ ╚████║██║╚██████╗██║  ██║███████╗                              //
//    ╚═╝     ╚═╝╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝ ╚═════╝╚═╝  ╚═╝╚══════╝                              //
//                                                                                                                  //
//    ██████╗ ██████╗  █████╗ ██╗  ██╗███████╗                                                                      //
//    ██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝██╔════╝                                                                      //
//    ██████╔╝██████╔╝███████║█████╔╝ █████╗                                                                        //
//    ██╔══██╗██╔══██╗██╔══██║██╔═██╗ ██╔══╝                                                                        //
//    ██████╔╝██║  ██║██║  ██║██║  ██╗███████╗                                                                      //
//    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝                                                                      //
//                                                                                                                  //
//==================================================================================================================//
//                                              MECHANICAL BRAKE SECTION                                            //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Mechanical Brake Parameters
/// @{
//--------------------------------------
//  Mechanical Brake Parameters
//--------------------------------------
/**
 * @brief Parameters for controlling an external mechanical brake.
 *
 * The mechanical brake is controlled by a PWM output. Release is triggered by setting RELEASE_BRAKE.
 * - BRAKE_RELEASING_DUTY_CYCLE: PWM duty cycle during release phase [%].
 * - BRAKE_HOLDING_DUTY_CYCLE: PWM duty cycle during holding phase [%].
 * - BRAKE_RELEASING_DURATION: Duration of release phase [ms].
 * - INVERT_BRAKE_OUTPUT: Invert brake output polarity (0: NORMAL, 1: INVERTED).
 *
 * Table — Mechanical Brake Parameters:
 *  NUMBER | NAME                      | DESCRIPTION
 *  ------ | ------------------------- | -----------------------------------------------------------------
 *   216   | RELEASE_BRAKE             | Release brake (apply PWM). 0: DEACTIVATED, 1: ACTIVATED. Default: 0.
 *   217   | BRAKE_RELEASING_DUTY_CYCLE| Duty cycle [%] for releasing phase. 0...99. Default: 75.
 *   218   | BRAKE_HOLDING_DUTY_CYCLE  | Duty cycle [%] for holding phase. 0...99. Default: 11.
 *   219   | BRAKE_RELEASING_DURATION  | Duration [ms] for releasing phase. 0...65535. Default: 80.
 *   221   | INVERT_BRAKE_OUTPUT       | Invert brake output. 0: NORMAL, 1: INVERTED. Default: 0.
 */
#define MECHANICAL_BRAKE_LIST(X) \
    X(RELEASE_BRAKE,             216, /*!< Release brake (apply PWM). 0: DEACTIVATED, 1: ACTIVATED. Default: 0. */) \
    X(BRAKE_RELEASING_DUTY_CYCLE, 217, /*!< Duty cycle [%] for releasing phase. 0...99. Default: 75. */) \
    X(BRAKE_HOLDING_DUTY_CYCLE,   218, /*!< Duty cycle [%] for holding phase. 0...99. Default: 11. */) \
    X(BRAKE_RELEASING_DURATION,   219, /*!< Duration [ms] for releasing phase. 0...65535. Default: 80. */) \
    X(INVERT_BRAKE_OUTPUT,        221, /*!< Invert brake output. 0: NORMAL, 1: INVERTED. Default: 0. */)

enum class MechanicalBrake : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    MECHANICAL_BRAKE_LIST(X)
    #undef X
};

inline const char* to_string(MechanicalBrake e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case MechanicalBrake::NAME: return #NAME;
        MECHANICAL_BRAKE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//     █████╗ ██╗   ██╗████████╗ ██████╗ ███╗   ███╗ █████╗ ████████╗██╗ ██████╗                                    //
//    ██╔══██╗██║   ██║╚══██╔══╝██╔═══██╗████╗ ████║██╔══██╗╚══██╔══╝██║██╔════╝                                    //
//    ███████║██║   ██║   ██║   ██║   ██║██╔████╔██║███████║   ██║   ██║██║                                         //
//    ██╔══██║██║   ██║   ██║   ██║   ██║██║╚██╔╝██║██╔══██║   ██║   ██║██║                                         //
//    ██║  ██║╚██████╔╝   ██║   ╚██████╔╝██║ ╚═╝ ██║██║  ██║   ██║   ██║╚██████╗                                    //
//    ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝     ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝                                    //
//                                                                                                                  //
//    ██╗  ██╗ ██████╗ ███╗   ███╗██╗███╗   ██╗ ██████╗                                                             //
//    ██║  ██║██╔═══██╗████╗ ████║██║████╗  ██║██╔════╝                                                             //
//    ███████║██║   ██║██╔████╔██║██║██╔██╗ ██║██║  ███╗                                                            //
//    ██╔══██║██║   ██║██║╚██╔╝██║██║██║╚██╗██║██║   ██║                                                            //
//    ██║  ██║╚██████╔╝██║ ╚═╝ ██║██║██║ ╚████║╚██████╔╝                                                            //
//    ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝ ╚═════╝                                                             //
//                                                                                                                  //
//==================================================================================================================//
//                                              AUTOMATIC HOMING SECTION                                            //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Reference Search (Automatic Homing) Parameters
/// @{
//--------------------------------------
//  Reference Search (Automatic Homing) Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring automatic homing/reference search routines.
 *
 * The TMC9660 supports eight different reference search patterns, configurable via REFERENCE_SWITCH_SEARCH_MODE.
 * Two speeds can be set: REFERENCE_SWITCH_SEARCH_SPEED (fast search) and REFERENCE_SWITCH_SPEED (slow, for accuracy).
 * Switch positions can be read out after search. The TMCL command RFS (13) is used to start/stop/query the search.
 *
 * Table — Reference Search (Automatic Homing) Parameters:
 *  NUMBER | NAME                          | DESCRIPTION
 *  ------ | ----------------------------- | -----------------------------------------------------------------
 *   165   | REFERENCE_SWITCH_SEARCH_MODE  | Reference search mode. See ReferenceSearchMode. Default: 0.
 *   166   | REFERENCE_SWITCH_SEARCH_SPEED | Speed for reference search [-134217728...134217727]. Default: 0.
 *   167   | REFERENCE_SWITCH_SPEED        | Lower speed for accurate switch positioning [-134217728...134217727]. Default: 0.
 *   168   | RIGHT_LIMIT_SWITCH_POSITION   | Position of right limit switch [-2147483648...2147483647]. Default: 0.
 *   169   | HOME_SWITCH_POSITION          | Position of home switch [-2147483648...2147483647]. Default: 0.
 *   170   | LAST_REFERENCE_POSITION       | Last reference position [-2147483648...2147483647]. Default: 0.
 */
#define REFERENCE_SEARCH_LIST(X) \
    X(REFERENCE_SWITCH_SEARCH_MODE,  165, /*!< Reference search mode. See ReferenceSearchMode. Default: 0. */) \
    X(REFERENCE_SWITCH_SEARCH_SPEED, 166, /*!< Speed for reference search [-134217728...134217727]. Default: 0. */) \
    X(REFERENCE_SWITCH_SPEED,        167, /*!< Lower speed for accurate switch positioning [-134217728...134217727]. Default: 0. */) \
    X(RIGHT_LIMIT_SWITCH_POSITION,   168, /*!< Position of right limit switch [-2147483648...2147483647]. Default: 0. */) \
    X(HOME_SWITCH_POSITION,          169, /*!< Position of home switch [-2147483648...2147483647]. Default: 0. */) \
    X(LAST_REFERENCE_POSITION,       170, /*!< Last reference position [-2147483648...2147483647]. Default: 0. */)

enum class ReferenceSearch : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SEARCH_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSearch e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case ReferenceSearch::NAME: return #NAME;
        REFERENCE_SEARCH_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Reference Search Modes
/// @{
//--------------------------------------
//  Reference Search Modes
//--------------------------------------
/**
 * @brief Enumerates reference search (homing) patterns.
 *
 * Table — Reference Search Modes:
 *  NUMBER | NAME                                   | DESCRIPTION
 *  ------ | ------------------------------------- | -----------------------------------------------------------------
 *     1   | LEFT_SWITCH                           | Search for left limit switch.
 *     2   | RIGHT_SWITCH_LEFT_SWITCH              | Search right limit, then left limit switch.
 *     3   | RIGHT_SWITCH_LEFT_SWITCH_BOTH_SIDES   | Right limit, then approach left limit from both sides.
 *     4   | LEFT_SWITCH_BOTH_SIDES                | Approach left limit from both sides.
 *     5   | HOME_SWITCH_NEG_DIR_LEFT_END_SWITCH   | Search home switch in negative direction, turn if left end detected.
 *     6   | HOME_SWITCH_POS_DIR_RIGHT_END_SWITCH  | Search home switch in positive direction, turn if right end detected.
 *     7   | HOME_SWITCH_NEG_DIR_IGNORE_END_SWITCH | Search home switch in negative direction, ignore end switch.
 *     8   | HOME_SWITCH_POS_DIR_IGNORE_END_SWITCH | Search home switch in positive direction, ignore end switch.
 */
#define REFERENCE_SEARCH_MODE_LIST(X) \
    X(LEFT_SWITCH,                           1, /*!< Search for left limit switch. */) \
    X(RIGHT_SWITCH_LEFT_SWITCH,              2, /*!< Search right limit, then left limit switch. */) \
    X(RIGHT_SWITCH_LEFT_SWITCH_BOTH_SIDES,   3, /*!< Right limit, then approach left limit from both sides. */) \
    X(LEFT_SWITCH_BOTH_SIDES,                4, /*!< Approach left limit from both sides. */) \
    X(HOME_SWITCH_NEG_DIR_LEFT_END_SWITCH,   5, /*!< Search home switch in negative direction, turn if left end detected. */) \
    X(HOME_SWITCH_POS_DIR_RIGHT_END_SWITCH,  6, /*!< Search home switch in positive direction, turn if right end detected. */) \
    X(HOME_SWITCH_NEG_DIR_IGNORE_END_SWITCH, 7, /*!< Search home switch in negative direction, ignore end switch. */) \
    X(HOME_SWITCH_POS_DIR_IGNORE_END_SWITCH, 8, /*!< Search home switch in positive direction, ignore end switch. */)

enum class ReferenceSwitchSearchMode : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SEARCH_MODE_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSwitchSearchMode e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case ReferenceSwitchSearchMode::NAME: return #NAME;
        REFERENCE_SEARCH_MODE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REFERENCE_SEARCH_MODE_LIST
/// @}

/// @name Reference Search TMCL Command Types
/// @{
//--------------------------------------
//  Reference Search TMCL Command Types
//--------------------------------------
/**
 * @brief Enumerates TMCL RFS (13) command types for reference search.
 *
 * Table — Reference Search TMCL Command Types:
 *  NUMBER | NAME   | DESCRIPTION
 *  ------ | ------ | ---------------------------
 *     0   | START  | Start reference search.
 *     1   | STOP   | Stop reference search.
 *     2   | STATUS | Return reference search status.
 */
#define REFERENCE_SEARCH_COMMAND_LIST(X) \
    X(START,  0, /*!< Start reference search. */) \
    X(STOP,   1, /*!< Stop reference search. */) \
    X(STATUS, 2, /*!< Return reference search status. */)

enum class ReferenceSearchCommand : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SEARCH_COMMAND_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSearchCommand e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case ReferenceSearchCommand::NAME: return #NAME;
        REFERENCE_SEARCH_COMMAND_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REFERENCE_SEARCH_COMMAND_LIST
/// @}

/// @name Reference Search Status Codes
/// @{
//--------------------------------------
//  Reference Search Status Codes
//--------------------------------------
/**
 * @brief Enumerates status codes for reference search state machine.
 *
 * Table — Reference Search Status Codes:
 *  NUMBER | NAME                                   | DESCRIPTION
 *  ------ | ------------------------------------- | -----------------------------------------------------------------
 *     0   | IDLE                                  | No description provided.
 *     1   | START_REFERENCE_DRIVE                 | No description provided.
 *     2   | START_DRIVE_TO_RIGHT_LIMIT_FAST       | No description provided.
 *     3   | WAIT_UNTIL_RIGHT_SWITCH_REACHED       | No description provided.
 *     4   | START_DRIVE_TO_LEFT_LIMIT_FAST        | No description provided.
 *     5   | WAIT_UNTIL_LEFT_SWITCH_REACHED        | No description provided.
 *     6   | DRIVE_OUT_OF_LEFT_SWITCH_SLOWLY       | No description provided.
 *     7   | WAIT_UNTIL_LEFT_SWITCH_EXITED_DRIVE_IN_AGAIN | No description provided.
 *     8   | WAIT_UNTIL_LEFT_SWITCH_REACHED_AGAIN_DRIVE_TO_POSITION | No description provided.
 *     9   | WAIT_UNTIL_POSITION_REACHED_SET_ZERO  | No description provided.
 *    10   | WAIT_UNTIL_SWITCH_PUSHED_AGAIN        | No description provided.
 *    11   | WAIT_UNTIL_OTHER_SIDE_SWITCH_REACHED  | No description provided.
 *    12   | RESERVED                              | No description provided.
 *    13   | WAIT_UNTIL_CENTER_SWITCH_REACHED      | No description provided.
 *    14   | REFERENCE_DRIVE_FINISHED_RESTORE_SETTINGS | No description provided.
 *    15   | STOP_REFERENCE_DRIVE                 | No description provided.
 */
#define REFERENCE_SEARCH_STATUS_LIST(X) \
    X(IDLE,                                  0, /*!< No description provided. */) \
    X(START_REFERENCE_DRIVE,                 1, /*!< No description provided. */) \
    X(START_DRIVE_TO_RIGHT_LIMIT_FAST,       2, /*!< No description provided. */) \
    X(WAIT_UNTIL_RIGHT_SWITCH_REACHED,       3, /*!< No description provided. */) \
    X(START_DRIVE_TO_LEFT_LIMIT_FAST,        4, /*!< No description provided. */) \
    X(WAIT_UNTIL_LEFT_SWITCH_REACHED,        5, /*!< No description provided. */) \
    X(DRIVE_OUT_OF_LEFT_SWITCH_SLOWLY,       6, /*!< No description provided. */) \
    X(WAIT_UNTIL_LEFT_SWITCH_EXITED_DRIVE_IN_AGAIN, 7, /*!< No description provided. */) \
    X(WAIT_UNTIL_LEFT_SWITCH_REACHED_AGAIN_DRIVE_TO_POSITION, 8, /*!< No description provided. */) \
    X(WAIT_UNTIL_POSITION_REACHED_SET_ZERO,  9, /*!< No description provided. */) \
    X(WAIT_UNTIL_SWITCH_PUSHED_AGAIN,        10, /*!< No description provided. */) \
    X(WAIT_UNTIL_OTHER_SIDE_SWITCH_REACHED,  11, /*!< No description provided. */) \
    X(RESERVED,                              12, /*!< No description provided. */) \
    X(WAIT_UNTIL_CENTER_SWITCH_REACHED,      13, /*!< No description provided. */) \
    X(REFERENCE_DRIVE_FINISHED_RESTORE_SETTINGS, 14, /*!< No description provided. */) \
    X(STOP_REFERENCE_DRIVE,                  15, /*!< No description provided. */)

enum class ReferenceSearchStatus : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REFERENCE_SEARCH_STATUS_LIST(X)
    #undef X
};

inline const char* to_string(ReferenceSearchStatus e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case ReferenceSearchStatus::NAME: return #NAME;
        REFERENCE_SEARCH_STATUS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REFERENCE_SEARCH_STATUS_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗████████╗███████╗██████╗     ██╗██████╗ ██╗██████╗                                                    //
//    ██╔════╝╚══██╔══╝██╔════╝██╔══██╗   ██╔╝██╔══██╗██║██╔══██╗                                                   //
//    ███████╗   ██║   █████╗  ██████╔╝  ██╔╝ ██║  ██║██║██████╔╝                                                   //
//    ╚════██║   ██║   ██╔══╝  ██╔═══╝  ██╔╝  ██║  ██║██║██╔══██╗                                                   //
//    ███████║   ██║   ███████╗██║     ██╔╝   ██████╔╝██║██║  ██║                                                   //
//    ╚══════╝   ╚═╝   ╚══════╝╚═╝     ╚═╝    ╚═════╝ ╚═╝╚═╝  ╚═╝                                                   //
//                                                                                                                  //
//==================================================================================================================//
//                                              STEP/DIR SECTION                                                    //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////
/// @name Step/Dir Interface Parameters
/// @{
//--------------------------------------
//  Step/Dir Interface Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring the STEP/DIR target movement interface.
 *
 * The STEP/DIR interface allows position control via step and direction signals. Supports micro-stepping,
 * extrapolation, and velocity feedforward. Micro-stepping is set by STEP_DIR_STEP_DIVIDER_SHIFT (1/1 to 1/1024).
 * Extrapolation smooths movement between steps and can be limited by velocity. Timeout and correction are configurable.
 *
 * NUMBER | NAME                                | DESCRIPTION
 * ------ | ----------------------------------- | ---------------------------------------------
 *    68  | VELOCITY_FEEDFORWARD_ENABLE         | Enable velocity feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 *   205  | STEP_DIR_STEP_DIVIDER_SHIFT         | Micro-step divider shift (see StepDirStepDividerShift). Default: 0 (full step). RWE
 *   206  | STEP_DIR_ENABLE                     | Enable STEP/DIR input. 0: DISABLED, 1: ENABLED. Default: 0. RW
 *   207  | STEP_DIR_EXTRAPOLATION_ENABLE       | Enable extrapolation. 0: DISABLED, 1: ENABLED. Default: 0. RW
 *   208  | STEP_DIR_STEP_SIGNAL_TIMEOUT_LIMIT  | Step signal timeout limit [ms]. 1...2000. Default: 1000. RW
 *   209  | STEP_DIR_MAXIMUM_EXTRAPOLATION_VELOCITY | Max velocity for extrapolation [eRPM]. 0...2147483647. Default: 2147483647. RW
 */
#define STEP_DIR_LIST(X) \
    /* VELOCITY_FEEDFORWARD_ENABLE defined in VELOCITY_CONTROL_LIST */ \
    X(STEP_DIR_STEP_DIVIDER_SHIFT, 205, /*!< Micro-step divider shift (see StepDirStepDividerShift). Default: 0 (full step). RWE */) \
    X(STEP_DIR_ENABLE, 206, /*!< Enable STEP/DIR input. 0: DISABLED, 1: ENABLED. Default: 0. RW */) \
    X(STEP_DIR_EXTRAPOLATION_ENABLE, 207, /*!< Enable extrapolation. 0: DISABLED, 1: ENABLED. Default: 0. RW */) \
    X(STEP_DIR_STEP_SIGNAL_TIMEOUT_LIMIT, 208, /*!< Step signal timeout limit [ms]. 1...2000. Default: 1000. RW */) \
    X(STEP_DIR_MAXIMUM_EXTRAPOLATION_VELOCITY, 209, /*!< Max velocity for extrapolation [eRPM]. 0...2147483647. Default: 2147483647. RW */)

enum class StepDir : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    STEP_DIR_LIST(X)
    #undef X
};

inline const char* to_string(StepDir e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case StepDir::NAME: return #NAME;
        STEP_DIR_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Step/Dir Microstep Divider Shift
/// @{
//--------------------------------------
//  Step/Dir Microstep Divider Shift
//--------------------------------------
/**
 * @brief Enumerates micro-step divider shift settings for STEP/DIR interface.
 *
 * Determines the number of micro-steps per full step (1/1 to 1/1024).
 *
 * NUMBER | NAME               | DESCRIPTION
 * ------ | ------------------ | --------------------------
 *     0  | STEP_MODE_FULL     | Full step (1/1)
 *     1  | STEP_MODE_HALF     | Half step (1/2)
 *     2  | STEP_MODE_QUARTER  | Quarter step (1/4)
 *     3  | STEP_MODE_1_8TH    | 1/8 step
 *     4  | STEP_MODE_1_16TH   | 1/16 step
 *     5  | STEP_MODE_1_32ND   | 1/32 step
 *     6  | STEP_MODE_1_64TH   | 1/64 step
 *     7  | STEP_MODE_1_128TH  | 1/128 step
 *     8  | STEP_MODE_1_256TH  | 1/256 step
 *     9  | STEP_MODE_1_512TH  | 1/512 step
 *    10  | STEP_MODE_1_1024TH | 1/1024 step
 */
#define STEP_DIR_STEP_DIVIDER_SHIFT_LIST(X) \
    X(STEP_MODE_FULL, 0, /*!< Full step (1/1) */) \
    X(STEP_MODE_HALF, 1, /*!< Half step (1/2) */) \
    X(STEP_MODE_QUARTER, 2, /*!< Quarter step (1/4) */) \
    X(STEP_MODE_1_8TH, 3, /*!< 1/8 step */) \
    X(STEP_MODE_1_16TH, 4, /*!< 1/16 step */) \
    X(STEP_MODE_1_32ND, 5, /*!< 1/32 step */) \
    X(STEP_MODE_1_64TH, 6, /*!< 1/64 step */) \
    X(STEP_MODE_1_128TH, 7, /*!< 1/128 step */) \
    X(STEP_MODE_1_256TH, 8, /*!< 1/256 step */) \
    X(STEP_MODE_1_512TH, 9, /*!< 1/512 step */) \
    X(STEP_MODE_1_1024TH, 10, /*!< 1/1024 step */)

enum class StepDirStepDividerShift : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    STEP_DIR_STEP_DIVIDER_SHIFT_LIST(X)
    #undef X
};

inline const char* to_string(StepDirStepDividerShift e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case StepDirStepDividerShift::NAME: return #NAME;
        STEP_DIR_STEP_DIVIDER_SHIFT_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef STEP_DIR_STEP_DIVIDER_SHIFT_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗  ██╗██╗██████╗ ███████╗██████╗ ███╗   ██╗ █████╗ ████████╗██╗ ██████╗ ███╗   ██╗                         //
//    ██║  ██║██║██╔══██╗██╔════╝██╔══██╗████╗  ██║██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║                         //
//    ███████║██║██████╔╝█████╗  ██████╔╝██╔██╗ ██║███████║   ██║   ██║██║   ██║██╔██╗ ██║                         //
//    ██╔══██║██║██╔══██╗██╔══╝  ██╔══██╗██║╚██╗██║██╔══██║   ██║   ██║██║   ██║██║╚██╗██║                         //
//    ██║  ██║██║██████╔╝███████╗██║  ██║██║ ╚████║██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║                         //
//    ╚═╝  ╚═╝╚═╝╚═════╝ ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝                         //
//                                                                                                                  //
//     █████╗ ███╗   ██╗██████╗     ██╗    ██╗ █████╗ ██╗  ██╗███████╗██╗   ██╗██████╗                             //
//    ██╔══██╗████╗  ██║██╔══██╗    ██║    ██║██╔══██╗██║ ██╔╝██╔════╝██║   ██║██╔══██╗                            //
//    ███████║██╔██╗ ██║██║  ██║    ██║ █╗ ██║███████║█████╔╝ █████╗  ██║   ██║██████╔╝                            //
//    ██╔══██║██║╚██╗██║██║  ██║    ██║███╗██║██╔══██║██╔═██╗ ██╔══╝  ██║   ██║██╔═══╝                             //
//    ██║  ██║██║ ╚████║██████╔╝    ╚███╔███╔╝██║  ██║██║  ██╗███████╗╚██████╔╝██║                                 //
//    ╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝      ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝ ╚═╝                                 //
//                                                                                                                  //
//==================================================================================================================//
//                                      HIBERNATION AND WAKEUP SECTION                                              //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Hibernation and Wakeup Parameters
/// @{
//--------------------------------------
//  Hibernation and Wakeup Parameters
//--------------------------------------
/**
 * @brief Parameters for configuring hibernation (low-power) and wakeup behavior.
 *
 * @warning These are **global bank-0** parameters (SGP/GGP), not axis parameters, and are not
 * part of ::Parameters. Prefer GlobalParamBank0::WAKE_PIN_CONTROL_ENABLE /
 * GO_TO_TIMEOUT_POWER_DOWN_STATE.
 *
 * The TMC9660 can be sent to a low-power hibernation state using WAKE_PIN_CONTROL_ENABLE or GO_TO_TIMEOUT_POWER_DOWN_STATE.
 * - WAKE_PIN_CONTROL_ENABLE enables the WAKE pin for entering/exiting power-down state.
 * - GO_TO_TIMEOUT_POWER_DOWN_STATE puts the device into hibernation for a predefined time.
 *   If the WAKE pin is configured, pulling it low powers down the device; pulling it high wakes it up.
 *
 * Table — Hibernation and Wakeup Parameters:
 *  NUMBER | NAME                      | DESCRIPTION
 *  ------ | ------------------------- | -----------------------------------------------------------------
 *    10   | WAKE_PIN_CONTROL_ENABLE   | Enable WAKE pin control. 0: DISABLED, 1: ENABLED. Default: 0. RWE
 *    11   | GO_TO_TIMEOUT_POWER_DOWN_STATE | Enter power-down for a predefined time. See PowerDownTimeout. Default: 0. W
 */
#define HIBERNATION_WAKEUP_LIST(X) \
    X(WAKE_PIN_CONTROL_ENABLE, 10, /*!< Enable WAKE pin control. 0: DISABLED, 1: ENABLED. Default: 0. RWE */) \
    X(GO_TO_TIMEOUT_POWER_DOWN_STATE, 11, /*!< Enter power-down for a predefined time. See PowerDownTimeout. Default: 0. W */)

enum class HibernationWakeup : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    HIBERNATION_WAKEUP_LIST(X)
    #undef X
};

inline const char* to_string(HibernationWakeup e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case HibernationWakeup::NAME: return #NAME;
        HIBERNATION_WAKEUP_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Power Down Timeout
/// @{
//--------------------------------------
//  Power Down Timeout
//--------------------------------------
/**
 * @brief Enumerates timeout durations for power-down state.
 *
 * Table — Power Down Timeout:
 *  NUMBER | NAME          | DESCRIPTION
 *  ------ | ------------- | -----------
 *     0   | T_250_MILLISEC | 250 ms
 *     1   | T_500_MILLISEC | 500 ms
 *     2   | T_1_SEC        | 1 second
 *     3   | T_2_SEC        | 2 seconds
 *     4   | T_4_SEC        | 4 seconds
 *     5   | T_8_SEC        | 8 seconds
 *     6   | T_16_SEC       | 16 seconds
 *     7   | T_32_SEC       | 32 seconds
 */
#define POWER_DOWN_TIMEOUT_LIST(X) \
    X(T_250_MILLISEC, 0, /*!< 250 ms */) \
    X(T_500_MILLISEC, 1, /*!< 500 ms */) \
    X(T_1_SEC, 2, /*!< 1 second */) \
    X(T_2_SEC, 3, /*!< 2 seconds */) \
    X(T_4_SEC, 4, /*!< 4 seconds */) \
    X(T_8_SEC, 5, /*!< 8 seconds */) \
    X(T_16_SEC, 6, /*!< 16 seconds */) \
    X(T_32_SEC, 7, /*!< 32 seconds */)

enum class PowerDownTimeout : uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    POWER_DOWN_TIMEOUT_LIST(X)
    #undef X
};

inline const char* to_string(PowerDownTimeout e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case PowerDownTimeout::NAME: return #NAME;
        POWER_DOWN_TIMEOUT_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef POWER_DOWN_TIMEOUT_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗███╗   ██╗████████╗███████╗██████╗ ███╗   ██╗ █████╗ ██╗                                                   //
//    ██║████╗  ██║╚══██╔══╝██╔════╝██╔══██╗████╗  ██║██╔══██╗██║                                                   //
//    ██║██╔██╗ ██║   ██║   █████╗  ██████╔╝██╔██╗ ██║███████║██║                                                   //
//    ██║██║╚██╗██║   ██║   ██╔══╝  ██╔══██╗██║╚██╗██║██╔══██║██║                                                   //
//    ██║██║ ╚████║   ██║   ███████╗██║  ██║██║ ╚████║██║  ██║███████╗                                              //
//    ╚═╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝                                              //
//                                                                                                                  //
//    ███╗   ███╗███████╗ █████╗ ███████╗██╗   ██╗██████╗ ███████╗███╗   ███╗███████╗███╗   ██╗████████╗███████╗   //
//    ████╗ ████║██╔════╝██╔══██╗██╔════╝██║   ██║██╔══██╗██╔════╝████╗ ████║██╔════╝████╗  ██║╚══██╔══╝██╔════╝   //
//    ██╔████╔██║█████╗  ███████║███████╗██║   ██║██████╔╝█████╗  ██╔████╔██║█████╗  ██╔██╗ ██║   ██║   ███████╗   //
//    ██║╚██╔╝██║██╔══╝  ██╔══██║╚════██║██║   ██║██╔══██╗██╔══╝  ██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║   ╚════██║   //
//    ██║ ╚═╝ ██║███████╗██║  ██║███████║╚██████╔╝██║  ██║███████╗██║ ╚═╝ ██║███████╗██║ ╚████║   ██║   ███████║   //
//    ╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝   //
//                                                                                                                  //
//==================================================================================================================//
//                                           INTERNAL MEASUREMENTS SECTION                                          //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name System Supply Parameters
/// @{
///--------------------------------------
/// System Supply Parameters
///--------------------------------------
/**
 * @brief Parameters for supply voltage warnings.
 * 
 * NUMBER | NAME                                | DESCRIPTION
 * ------ | ----------------------------------- | -----------
 *   290  | SUPPLY_VOLTAGE                     | Actual supply voltage in 0.1 V units. Read-only.
 *   291  | SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD | Supply overvoltage warning threshold [0…1000]. RWE
 *   292  | SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD | Supply undervoltage warning threshold [0…1000]. RWE
 */
#define SYSTEM_STATUS_SUPPLY_LIST(X) \
    X(SUPPLY_VOLTAGE, 290, /*!< Actual supply voltage in 0.1 V units. Read-only. */) \
    X(SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD, 291, /*!< Supply overvoltage warning threshold [0…1000]. RWE */) \
    X(SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD, 292, /*!< Supply undervoltage warning threshold [0…1000]. RWE */)

enum class SystemStatusSupply : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    SYSTEM_STATUS_SUPPLY_LIST(X)
    #undef X
};

inline const char* to_string(SystemStatusSupply e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case SystemStatusSupply::NAME: return #NAME;
        SYSTEM_STATUS_SUPPLY_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Internal Measurement Parameters
/// @{
///--------------------------------------
/// Internal Measurement Parameters
///--------------------------------------
/**
 * @brief Raw diagnostic values and FOC internal measurements.
 * 
 * These parameters provide access to interim results and measurements from the Field-Oriented Control (FOC)
 * algorithm, as well as raw input states. They are useful for debugging and diagnostics.
 * 
 * NUMBER | NAME                                | DESCRIPTION
 * ------ | ----------------------------------- | -----------
 *   304  | MCC_INPUTS_RAW                     | Raw inputs for ABN, hall, reference switches, driver enabled, hall filtered and ABN2 or Step/Dir [0...32767]. Read-only.
 *   305  | FOC_VOLTAGE_UX                     | Interim result of the FOC for phase U (X in case of stepper motor) [-32768...32767]. Read-only.
 *   306  | FOC_VOLTAGE_WY                     | Interim result of the FOC for phase W (Y in case of stepper motor) [-32768...32767]. Read-only.
 *   307  | FOC_VOLTAGE_V                      | Interim result of the FOC for phase V (BLDC motor only) [-32768...32767]. Read-only.
 *   308  | FIELDWEAKENING_I                   | I parameter for field weakening controller [0...32767]. Default: 0. RWE
 *   310  | FIELDWEAKENING_VOLTAGE_THRESHOLD   | Maximum motor voltage allowed for field weakening [0...32767]. Default: 32767. RWE
 *   311  | FOC_CURRENT_UX                     | Interim measurement of the FOC for phase UX [-32768...32767]. Read-only.
 *   312  | FOC_CURRENT_V                      | Interim measurement of the FOC for phase V [-32768...32767]. Read-only.
 *   313  | FOC_CURRENT_WY                     | Interim measurement of the FOC for phase WY [-32768...32767]. Read-only.
 *   314  | FOC_VOLTAGE_UQ                     | Interim measurement of the FOC for Uq [-32768...32767]. Read-only.
 *   315  | FOC_CURRENT_IQ                     | Interim measurement of the FOC for Iq [-32768...32767]. Read-only.
 */
#define INTERNAL_MEASUREMENT_LIST(X) \
    X(MCC_INPUTS_RAW, 304, /*!< Raw inputs for ABN, hall, reference switches, driver enabled, hall filtered and ABN2 or Step/Dir [0...32767]. Read-only. */) \
    X(FOC_VOLTAGE_UX, 305, /*!< Interim result of the FOC for phase U (X in case of stepper motor) [-32768...32767]. Read-only. */) \
    X(FOC_VOLTAGE_WY, 306, /*!< Interim result of the FOC for phase W (Y in case of stepper motor) [-32768...32767]. Read-only. */) \
    X(FOC_VOLTAGE_V, 307, /*!< Interim result of the FOC for phase V (BLDC motor only) [-32768...32767]. Read-only. */) \
    /* FIELDWEAKENING_I and FIELDWEAKENING_VOLTAGE_THRESHOLD defined in TORQUE_FLUX_CONTROL_LIST */ \
    X(FOC_CURRENT_UX, 311, /*!< Interim measurement of the FOC for phase UX [-32768...32767]. Read-only. */) \
    X(FOC_CURRENT_V, 312, /*!< Interim measurement of the FOC for phase V [-32768...32767]. Read-only. */) \
    X(FOC_CURRENT_WY, 313, /*!< Interim measurement of the FOC for phase WY [-32768...32767]. Read-only. */) \
    X(FOC_VOLTAGE_UQ, 314, /*!< Interim measurement of the FOC for Uq [-32768...32767]. Read-only. */) \
    X(FOC_CURRENT_IQ, 315, /*!< Interim measurement of the FOC for Iq [-32768...32767]. Read-only. */)

enum class InternalMeasurement : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    INTERNAL_MEASUREMENT_LIST(X)
    #undef X
};

inline const char* to_string(InternalMeasurement e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case InternalMeasurement::NAME: return #NAME;
        INTERNAL_MEASUREMENT_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Combined Diagnostic Values
/// @{
///--------------------------------------
/// Combined Diagnostic Values
///--------------------------------------
/**
 * @brief Simplified combined measurement registers used during tuning.
 * 
 * These parameters provide compact diagnostic values primarily used during motor tuning operations.
 * They combine multiple measurements into single 32-bit values or provide integrated measurements
 * to facilitate data collection at lower sampling rates.
 * 
 * NUMBER | NAME                                | DESCRIPTION
 * ------ | ----------------------------------- | -----------
 *   330  | TORQUE_FLUX_COMBINED_TARGET_VALUES | Raw (unscaled) torque and flux target values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only.
 *   331  | TORQUE_FLUX_COMBINED_ACTUAL_VALUES | Raw (unscaled) torque and flux actual values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only.
 *   332  | VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES | Raw (unscaled) voltage actual values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only.
 *   333  | INTEGRATED_ACTUAL_TORQUE_VALUE     | Periodically summed up actual torque value. Used for simplified measurement with low measurement frequency during tuning operations. [0...4294967295]. Read-only.
 *   334  | INTEGRATED_ACTUAL_VELOCITY_VALUE   | Periodically summed up actual velocity value. Used for simplified measurement with low measurement frequency during tuning operations. [0...4294967295]. Read-only.
 */
#define COMBINED_DIAGNOSTIC_VALUES_LIST(X) \
    X(TORQUE_FLUX_COMBINED_TARGET_VALUES, 330, /*!< Raw (unscaled) torque and flux target values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only. */) \
    X(TORQUE_FLUX_COMBINED_ACTUAL_VALUES, 331, /*!< Raw (unscaled) torque and flux actual values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only. */) \
    X(VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES, 332, /*!< Raw (unscaled) voltage actual values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only. */) \
    X(INTEGRATED_ACTUAL_TORQUE_VALUE, 333, /*!< Periodically summed up actual torque value. Used for simplified measurement with low measurement frequency during tuning operations. [0...4294967295]. Read-only. */) \
    X(INTEGRATED_ACTUAL_VELOCITY_VALUE, 334, /*!< Periodically summed up actual velocity value. Used for simplified measurement with low measurement frequency during tuning operations. [0...4294967295]. Read-only. */)

enum class CombinedDiagnosticValues : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    COMBINED_DIAGNOSTIC_VALUES_LIST(X)
    #undef X
};

inline const char* to_string(CombinedDiagnosticValues e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case CombinedDiagnosticValues::NAME: return #NAME;
        COMBINED_DIAGNOSTIC_VALUES_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗██████╗ ██████╗  ██████╗ ██████╗ ███████╗         ███████╗██╗      █████╗  ██████╗ ███████╗           //
//    ██╔════╝██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔════╝         ██╔════╝██║     ██╔══██╗██╔════╝ ██╔════╝           //
//    █████╗  ██████╔╝██████╔╝██║   ██║██████╔╝███████╗  █████╗ █████╗  ██║     ███████║██║  ███╗███████╗           //
//    ██╔══╝  ██╔══██╗██╔══██╗██║   ██║██╔══██╗╚════██║  ╚════╝ ██╔══╝  ██║     ██╔══██║██║   ██║╚════██║           //
//    ███████╗██║  ██║██║  ██║╚██████╔╝██║  ██║███████║         ██║     ███████╗██║  ██║╚██████╔╝███████║           //
//    ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝         ╚═╝     ╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝           //
//                                                                                                                  //
//==================================================================================================================//
//                                              ERRORS & FLAGS SECTION                                              //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Temperature and Error Flags Parameters
/// @{
//--------------------------------------
//  Temperature and Error Flags Parameters
//--------------------------------------
/**
 * @brief Parameters for temperature monitoring and error flags.
 *
 * NUMBER | NAME                                | DESCRIPTION
 * ------ | ----------------------------------- | -----------
 *   289  | GENERAL_STATUS_FLAGS               | General status flags. See GeneralStatusFlags enum. Read-only.
 *   293  | EXTERNAL_TEMPERATURE               | External temperature sensor reading [0-65535]. Read-only.
 *   294  | EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD | Shutdown threshold for external temperature [0-65535]. Default: 65535.
 *   295  | EXTERNAL_TEMPERATURE_WARNING_THRESHOLD  | Warning threshold for external temperature [0-65535]. Default: 65535.
 *   296  | CHIP_TEMPERATURE                   | Chip temperature reading [0-65535]. Read-only.
 *   297  | CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD | Shutdown threshold for chip temperature [0-65535]. Default: 65535.
 *   298  | CHIP_TEMPERATURE_WARNING_THRESHOLD  | Warning threshold for chip temperature [0-65535]. Default: 65535.
 *   299  | GENERAL_ERROR_FLAGS                | General error flags. See GeneralErrorFlags enum. Read-only.
 *   300  | GDRV_ERROR_FLAGS                   | Gate driver error flags. See GateDriverErrorFlags enum. Read-only.
 *   301  | ADC_STATUS_FLAGS                   | ADC status flags. See AdcStatusFlags enum. Write-to-clear.
 */
#define ERRORS_AND_FLAGS_LIST(X) \
    X(GENERAL_STATUS_FLAGS, 289, /*!< General status flags. See GeneralStatusFlags enum. Read-only. */) \
    /* Temperature parameters defined in TEMPERATURE_PROTECTION_LIST */ \
    X(GENERAL_ERROR_FLAGS, 299, /*!< General error flags. See GeneralErrorFlags enum. Read-only. */) \
    X(GDRV_ERROR_FLAGS, 300, /*!< Gate driver error flags. See GateDriverErrorFlags enum. Read-only. */) \
    X(ADC_STATUS_FLAGS, 301, /*!< ADC status flags. See AdcStatusFlags enum. Write-to-clear. */)

enum class ErrorsAndFlags : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ERRORS_AND_FLAGS_LIST(X)
    #undef X
};

inline const char* to_string(ErrorsAndFlags e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case ErrorsAndFlags::NAME: return #NAME;
        ERRORS_AND_FLAGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

#define FLAG(bit) (1u << (bit)) /** Bit flag macro */

/// @name General Status Flags
/// @{
//--------------------------------------
//  General Status Flags
//--------------------------------------
/**
 * @brief General status flags indicating system state, events, and hardware availability.
 * 
 * This 32-bit status register contains flags that provide information about the current system state,
 * regulation modes, configuration status, hardware events, and available peripherals.
 * 
 * Most flags are read-only (R), while some can be read, written, and cleared (RWC).
 * Flags marked as RWC can be cleared by writing a 1 to the corresponding bit.
 * 
 * NUMBER | NAME                          | DESCRIPTION
 * ------ | ----------------------------- | -----------
 * FLAG(0)  | REGULATION_STOPPED            | System does not regulate motion. Read-only.
 * FLAG(1)  | REGULATION_TORQUE             | System is regulating mode torque. Read-only.
 * FLAG(2)  | REGULATION_VELOCITY           | System is regulating mode velocity. Read-only.
 * FLAG(3)  | REGULATION_POSITION           | System is regulating mode position. Read-only.
 * FLAG(4)  | CONFIG_STORED                 | Config was stored successfully. Read-write-clear.
 * FLAG(5)  | CONFIG_LOADED                 | Config was loaded successfully. Read-write-clear.
 * FLAG(6)  | CONFIG_READ_ONLY              | Memory for config is read only. Read-only.
 * FLAG(7)  | TMCL_SCRIPT_READ_ONLY         | Memory for TMCL script is read only. Read-only.
 * FLAG(8)  | BRAKE_CHOPPER_ACTIVE          | Brake chopper is active. Read-only.
 * FLAG(9)  | POSITION_REACHED              | Actual velocity and target velocity are below POSITION_REACHED_THRESHOLD. Read-only.
 * FLAG(10) | VELOCITY_REACHED              | Actual velocity and target velocity are below VELOCITY_REACHED_THRESHOLD. Read-only.
 * FLAG(11) | ADC_OFFSET_CALIBRATED         | The ADC offset was calibrated automatically (clear to recalibrate). Read-write-clear.
 * FLAG(12) | RAMPER_LATCHED                | The ramper latched a position. Read-write-clear.
 * FLAG(13) | RAMPER_EVENT_STOP_SWITCH      | Ramper had a switch stop event. Read-only.
 * FLAG(14) | RAMPER_EVENT_STOP_DEVIATION   | Ramper had a deviation stop event. Read-write-clear.
 * FLAG(15) | RAMPER_VELOCITY_REACHED       | The ramper reached its velocity target. Read-only.
 * FLAG(16) | RAMPER_POSITION_REACHED       | The ramper reached its position target. Read-only.
 * FLAG(17) | RAMPER_SECOND_MOVE            | The ramper needed a second move to reach target. Read-write-clear.
 * FLAG(18) | IIT_1_ACTIVE                  | IIT 1 active. Read-only.
 * FLAG(19) | IIT_2_ACTIVE                  | IIT 2 active. Read-only.
 * FLAG(20) | REFSEARCH_FINISHED            | Reference search finished. Read-only.
 * FLAG(21) | Y2_USED_FOR_BRAKING           | Fourth phase used for braking. Read-only.
 * FLAG(23) | STEPDIR_INPUT_AVAILABLE       | Signals that StepDir is available. Read-only.
 * FLAG(24) | RIGHT_REF_SWITCH_AVAILABLE    | Signals that REF_R is available. Read-only.
 * FLAG(25) | HOME_REF_SWITCH_AVAILABLE     | Signals that REF_H is available. Read-only.
 * FLAG(26) | LEFT_REF_SWITCH_AVAILABLE     | Signals that REF_L is available. Read-only.
 * FLAG(27) | ABN2_FEEDBACK_AVAILABLE       | Signals that ABN2 feedback is available. Read-only.
 * FLAG(28) | HALL_FEEDBACK_AVAILABLE       | Signals that hall feedback is available. Read-only.
 * FLAG(29) | ABN1_FEEDBACK_AVAILABLE       | Signals that ABN1 feedback is available. Read-only.
 * FLAG(30) | SPI_FLASH_AVAILABLE           | Signals that an external SPI flash is available. Read-only.
 * FLAG(31) | I2C_EEPROM_AVAILABLE          | Signals that an external I2C EEPROM is available. Read-only.
 */
#define GENERAL_STATUS_FLAGS_LIST(X) \
    X(REGULATION_STOPPED,              FLAG(0),  /*!< System does not regulate motion. Read-only. */) \
    X(REGULATION_TORQUE,               FLAG(1),  /*!< System is regulating mode torque. Read-only. */) \
    X(REGULATION_VELOCITY,             FLAG(2),  /*!< System is regulating mode velocity. Read-only. */) \
    X(REGULATION_POSITION,             FLAG(3),  /*!< System is regulating mode position. Read-only. */) \
    X(CONFIG_STORED,                   FLAG(4),  /*!< Config was stored successfully. Read-write-clear. */) \
    X(CONFIG_LOADED,                   FLAG(5),  /*!< Config was loaded successfully. Read-write-clear. */) \
    X(CONFIG_READ_ONLY,                FLAG(6),  /*!< Memory for config is read only. Read-only. */) \
    X(TMCL_SCRIPT_READ_ONLY,           FLAG(7),  /*!< Memory for TMCL script is read only. Read-only. */) \
    X(BRAKE_CHOPPER_ACTIVE,            FLAG(8),  /*!< Brake chopper is active. Read-only. */) \
    X(POSITION_REACHED,                FLAG(9),  /*!< Actual velocity and target velocity are below POSITION_REACHED_THRESHOLD. Read-only. */) \
    X(VELOCITY_REACHED,                FLAG(10), /*!< Actual velocity and target velocity are below VELOCITY_REACHED_THRESHOLD. Read-only. */) \
    X(ADC_OFFSET_CALIBRATED,           FLAG(11), /*!< The ADC offset was calibrated automatically (clear to recalibrate). Read-write-clear. */) \
    X(RAMPER_LATCHED,                  FLAG(12), /*!< The ramper latched a position. Read-write-clear. */) \
    X(RAMPER_EVENT_STOP_SWITCH,        FLAG(13), /*!< Ramper had a switch stop event. Read-only. */) \
    X(RAMPER_EVENT_STOP_DEVIATION,     FLAG(14), /*!< Ramper had a deviation stop event. Read-write-clear. */) \
    X(RAMPER_VELOCITY_REACHED,         FLAG(15), /*!< The ramper reached its velocity target. Read-only. */) \
    X(RAMPER_POSITION_REACHED,         FLAG(16), /*!< The ramper reached its position target. Read-only. */) \
    X(RAMPER_SECOND_MOVE,              FLAG(17), /*!< The ramper needed a second move to reach target. Read-write-clear. */) \
    X(IIT_1_ACTIVE,                    FLAG(18), /*!< IIT 1 active. Read-only. */) \
    X(IIT_2_ACTIVE,                    FLAG(19), /*!< IIT 2 active. Read-only. */) \
    X(REFSEARCH_FINISHED,              FLAG(20), /*!< Reference search finished. Read-only. */) \
    X(Y2_USED_FOR_BRAKING,             FLAG(21), /*!< Fourth phase used for braking. Read-only. */) \
    X(STEPDIR_INPUT_AVAILABLE,         FLAG(23), /*!< Signals that StepDir is available. Read-only. */) \
    X(RIGHT_REF_SWITCH_AVAILABLE,      FLAG(24), /*!< Signals that REF_R is available. Read-only. */) \
    X(HOME_REF_SWITCH_AVAILABLE,       FLAG(25), /*!< Signals that REF_H is available. Read-only. */) \
    X(LEFT_REF_SWITCH_AVAILABLE,       FLAG(26), /*!< Signals that REF_L is available. Read-only. */) \
    X(ABN2_FEEDBACK_AVAILABLE,         FLAG(27), /*!< Signals that ABN2 feedback is available. Read-only. */) \
    X(HALL_FEEDBACK_AVAILABLE,         FLAG(28), /*!< Signals that hall feedback is available. Read-only. */) \
    X(ABN1_FEEDBACK_AVAILABLE,         FLAG(29), /*!< Signals that ABN1 feedback is available. Read-only. */) \
    X(SPI_FLASH_AVAILABLE,             FLAG(30), /*!< Signals that an external SPI flash is available. Read-only. */) \
    X(I2C_EEPROM_AVAILABLE,            FLAG(31), /*!< Signals that an external I2C EEPROM is available. Read-only. */)

enum class GeneralStatusFlags : uint32_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GENERAL_STATUS_FLAGS_LIST(X)
    #undef X
};

inline const char* to_string(GeneralStatusFlags e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case GeneralStatusFlags::NAME: return #NAME;
        GENERAL_STATUS_FLAGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GENERAL_STATUS_FLAGS_LIST
/// @}

/// @name General Error Flags
/// @{
//--------------------------------------
//  General Error Flags
//--------------------------------------
/**
 * @brief Enumerates general error flags for GENERAL_ERROR_FLAGS.
 * 
 * These flags indicate various error conditions in the system. Most of these flags
 * are Read-Write-Clear (RWC), meaning they can be cleared by writing a 1 to the 
 * corresponding bit position.
 * 
 * NUMBER | NAME                          | DESCRIPTION
 * ------ | ----------------------------- | -----------
 * FLAG(0)  | CONFIG_ERROR                 | Verification of config storage failed. Read-only.
 * FLAG(1)  | TMCL_SCRIPT_ERROR            | TMCL Script not available. Read-only.
 * FLAG(2)  | HOMESWITCH_NOT_FOUND         | Reference search for home switch failed. Read-only.
 * FLAG(5)  | HALL_ERROR                   | Signals an invalid hall state. Read-write-clear.
 * FLAG(9)  | WATCHDOG_EVENT               | Watchdog reset indication. Read-write-clear.
 * FLAG(13) | EXT_TEMP_EXCEEDED            | External temperature exceeded. Read-write-clear.
 * FLAG(14) | CHIP_TEMP_EXCEEDED           | Chip temperature threshold exceeded. Read-write-clear.
 * FLAG(16) | I2T_1_EXCEEDED               | Signals that I²t limit 1 was exceeded. Read-write-clear.
 * FLAG(17) | I2T_2_EXCEEDED               | Signals that I²t limit 2 was exceeded. Read-write-clear.
 * FLAG(18) | EXT_TEMP_WARNING             | External temperature warning threshold exceeded. Read-write-clear.
 * FLAG(19) | SUPPLY_OVERVOLTAGE_WARNING   | Supply overvoltage warning threshold exceeded. Read-write-clear.
 * FLAG(20) | SUPPLY_UNDERVOLTAGE_WARNING  | Supply voltage below undervoltage warning threshold. Read-write-clear.
 * FLAG(21) | ADC_IN_OVERVOLTAGE           | ADC IN over 2V while ADC enabled. Read-write-clear.
 * FLAG(22) | FAULT_RETRY_HAPPENED         | The set number of max. retries was exceeded without recovering. Read-write-clear.
 * FLAG(23) | FAULT_RETRIES_FAILED         | All retries of a detected fault failed. Read-write-clear.
 * FLAG(24) | CHIP_TEMP_WARNING            | Chip temperature warning threshold exceeded. Read-write-clear.
 * FLAG(26) | HEARTBEAT_STOPPED            | Heartbeat stopped. Read-write-clear.
 */
#define GENERAL_ERROR_FLAGS_LIST(X) \
    X(CONFIG_ERROR,                 FLAG(0),  /*!< Verification of config storage failed. Read-only. */) \
    X(TMCL_SCRIPT_ERROR,            FLAG(1),  /*!< TMCL Script not available. Read-only. */) \
    X(HOMESWITCH_NOT_FOUND,         FLAG(2),  /*!< Reference search for home switch failed. Read-only. */) \
    X(HALL_ERROR,                   FLAG(5),  /*!< Signals an invalid hall state. Read-write-clear. */) \
    X(WATCHDOG_EVENT,               FLAG(9),  /*!< Watchdog reset indication. Read-write-clear. */) \
    X(EXT_TEMP_EXCEEDED,            FLAG(13), /*!< External temperature exceeded. Read-write-clear. */) \
    X(CHIP_TEMP_EXCEEDED,           FLAG(14), /*!< Chip temperature threshold exceeded. Read-write-clear. */) \
    X(I2T_1_EXCEEDED,               FLAG(16), /*!< Signals that I²t limit 1 was exceeded. Read-write-clear. */) \
    X(I2T_2_EXCEEDED,               FLAG(17), /*!< Signals that I²t limit 2 was exceeded. Read-write-clear. */) \
    X(EXT_TEMP_WARNING,             FLAG(18), /*!< External temperature warning threshold exceeded. Read-write-clear. */) \
    X(SUPPLY_OVERVOLTAGE_WARNING,   FLAG(19), /*!< Supply overvoltage warning threshold exceeded. Read-write-clear. */) \
    X(SUPPLY_UNDERVOLTAGE_WARNING,  FLAG(20), /*!< Supply voltage below undervoltage warning threshold. Read-write-clear. */) \
    X(ADC_IN_OVERVOLTAGE,           FLAG(21), /*!< ADC IN over 2V while ADC enabled. Read-write-clear. */) \
    X(FAULT_RETRY_HAPPENED,         FLAG(22), /*!< The set number of max. retries was exceeded without recovering. Read-write-clear. */) \
    X(FAULT_RETRIES_FAILED,         FLAG(23), /*!< All retries of a detected fault failed. Read-write-clear. */) \
    X(CHIP_TEMP_WARNING,            FLAG(24), /*!< Chip temperature warning threshold exceeded. Read-write-clear. */) \
    X(HEARTBEAT_STOPPED,            FLAG(26), /*!< Heartbeat stopped. Read-write-clear. */)

enum class GeneralErrorFlags : uint32_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GENERAL_ERROR_FLAGS_LIST(X)
    #undef X
};

inline const char* to_string(GeneralErrorFlags e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case GeneralErrorFlags::NAME: return #NAME;
        GENERAL_ERROR_FLAGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GENERAL_ERROR_FLAGS_LIST
/// @}

/// @name Gate Driver Error Flags
/// @{
//--------------------------------------
//  Gate Driver Error Flags
//--------------------------------------
/**
 * @brief Enumerates gate driver error flags for GDRV_ERROR_FLAGS.
 *
 * NUMBER | NAME                          | DESCRIPTION
 * ------ | ----------------------------- | -----------
 * FLAG(0)  | U_LOW_SIDE_OVERCURRENT        | U low side overcurrent.
 * FLAG(1)  | V_LOW_SIDE_OVERCURRENT        | V low side overcurrent.
 * FLAG(2)  | W_LOW_SIDE_OVERCURRENT        | W low side overcurrent.
 * FLAG(3)  | Y2_LOW_SIDE_OVERCURRENT       | Y2 low side overcurrent.
 * FLAG(4)  | U_LOW_SIDE_DISCHARGE_SHORT    | U low side discharge short.
 * FLAG(5)  | V_LOW_SIDE_DISCHARGE_SHORT    | V low side discharge short.
 * FLAG(6)  | W_LOW_SIDE_DISCHARGE_SHORT    | W low side discharge short.
 * FLAG(7)  | Y2_LOW_SIDE_DISCHARGE_SHORT   | Y2 low side discharge short.
 * FLAG(8)  | U_LOW_SIDE_CHARGE_SHORT       | U low side charge short.
 * FLAG(9)  | V_LOW_SIDE_CHARGE_SHORT       | V low side charge short.
 * FLAG(10) | W_LOW_SIDE_CHARGE_SHORT       | W low side charge short.
 * FLAG(11) | Y2_LOW_SIDE_CHARGE_SHORT      | Y2 low side charge short.
 * FLAG(12) | U_BOOTSTRAP_UNDERVOLTAGE      | U bootstrap undervoltage.
 * FLAG(13) | V_BOOTSTRAP_UNDERVOLTAGE      | V bootstrap undervoltage.
 * FLAG(14) | W_BOOTSTRAP_UNDERVOLTAGE      | W bootstrap undervoltage.
 * FLAG(15) | Y2_BOOTSTRAP_UNDERVOLTAGE     | Y2 bootstrap undervoltage.
 * FLAG(16) | U_HIGH_SIDE_OVERCURRENT       | U high side overcurrent.
 * FLAG(17) | V_HIGH_SIDE_OVERCURRENT       | V high side overcurrent.
 * FLAG(18) | W_HIGH_SIDE_OVERCURRENT       | W high side overcurrent.
 * FLAG(19) | Y2_HIGH_SIDE_OVERCURRENT      | Y2 high side overcurrent.
 * FLAG(20) | U_HIGH_SIDE_DISCHARGE_SHORT   | U high side discharge short.
 * FLAG(21) | V_HIGH_SIDE_DISCHARGE_SHORT   | V high side discharge short.
 * FLAG(22) | W_HIGH_SIDE_DISCHARGE_SHORT   | W high side discharge short.
 * FLAG(23) | Y2_HIGH_SIDE_DISCHARGE_SHORT  | Y2 high side discharge short.
 * FLAG(24) | U_HIGH_SIDE_CHARGE_SHORT      | U high side charge short.
 * FLAG(25) | V_HIGH_SIDE_CHARGE_SHORT      | V high side charge short.
 * FLAG(26) | W_HIGH_SIDE_CHARGE_SHORT      | W high side charge short.
 * FLAG(27) | Y2_HIGH_SIDE_CHARGE_SHORT     | Y2 high side charge short.
 * FLAG(29) | GDRV_UNDERVOLTAGE             | Gate driver undervoltage.
 * FLAG(30) | GDRV_LOW_VOLTAGE              | Gate driver low voltage.
 * FLAG(31) | GDRV_SUPPLY_UNDERVOLTAGE      | Gate driver supply undervoltage.
 */
#define GATE_DRIVER_ERROR_FLAGS_LIST(X) \
    X(U_LOW_SIDE_OVERCURRENT,        FLAG(0),  /*!< U low side overcurrent. */) \
    X(V_LOW_SIDE_OVERCURRENT,        FLAG(1),  /*!< V low side overcurrent. */) \
    X(W_LOW_SIDE_OVERCURRENT,        FLAG(2),  /*!< W low side overcurrent. */) \
    X(Y2_LOW_SIDE_OVERCURRENT,       FLAG(3),  /*!< Y2 low side overcurrent. */) \
    X(U_LOW_SIDE_DISCHARGE_SHORT,    FLAG(4),  /*!< U low side discharge short. */) \
    X(V_LOW_SIDE_DISCHARGE_SHORT,    FLAG(5),  /*!< V low side discharge short. */) \
    X(W_LOW_SIDE_DISCHARGE_SHORT,    FLAG(6),  /*!< W low side discharge short. */) \
    X(Y2_LOW_SIDE_DISCHARGE_SHORT,   FLAG(7),  /*!< Y2 low side discharge short. */) \
    X(U_LOW_SIDE_CHARGE_SHORT,       FLAG(8),  /*!< U low side charge short. */) \
    X(V_LOW_SIDE_CHARGE_SHORT,       FLAG(9),  /*!< V low side charge short. */) \
    X(W_LOW_SIDE_CHARGE_SHORT,       FLAG(10), /*!< W low side charge short. */) \
    X(Y2_LOW_SIDE_CHARGE_SHORT,      FLAG(11), /*!< Y2 low side charge short. */) \
    X(U_BOOTSTRAP_UNDERVOLTAGE,      FLAG(12), /*!< U bootstrap undervoltage. */) \
    X(V_BOOTSTRAP_UNDERVOLTAGE,      FLAG(13), /*!< V bootstrap undervoltage. */) \
    X(W_BOOTSTRAP_UNDERVOLTAGE,      FLAG(14), /*!< W bootstrap undervoltage. */) \
    X(Y2_BOOTSTRAP_UNDERVOLTAGE,     FLAG(15), /*!< Y2 bootstrap undervoltage. */) \
    X(U_HIGH_SIDE_OVERCURRENT,       FLAG(16), /*!< U high side overcurrent. */) \
    X(V_HIGH_SIDE_OVERCURRENT,       FLAG(17), /*!< V high side overcurrent. */) \
    X(W_HIGH_SIDE_OVERCURRENT,       FLAG(18), /*!< W high side overcurrent. */) \
    X(Y2_HIGH_SIDE_OVERCURRENT,      FLAG(19), /*!< Y2 high side overcurrent. */) \
    X(U_HIGH_SIDE_DISCHARGE_SHORT,   FLAG(20), /*!< U high side discharge short. */) \
    X(V_HIGH_SIDE_DISCHARGE_SHORT,   FLAG(21), /*!< V high side discharge short. */) \
    X(W_HIGH_SIDE_DISCHARGE_SHORT,   FLAG(22), /*!< W high side discharge short. */) \
    X(Y2_HIGH_SIDE_DISCHARGE_SHORT,  FLAG(23), /*!< Y2 high side discharge short. */) \
    X(U_HIGH_SIDE_CHARGE_SHORT,      FLAG(24), /*!< U high side charge short. */) \
    X(V_HIGH_SIDE_CHARGE_SHORT,      FLAG(25), /*!< V high side charge short. */) \
    X(W_HIGH_SIDE_CHARGE_SHORT,      FLAG(26), /*!< W high side charge short. */) \
    X(Y2_HIGH_SIDE_CHARGE_SHORT,     FLAG(27), /*!< Y2 high side charge short. */) \
    X(GDRV_UNDERVOLTAGE,             FLAG(29), /*!< Gate driver undervoltage. */) \
    X(GDRV_LOW_VOLTAGE,              FLAG(30), /*!< Gate driver low voltage. */) \
    X(GDRV_SUPPLY_UNDERVOLTAGE,      FLAG(31), /*!< Gate driver supply undervoltage. */)

enum class GateDriverErrorFlags : uint32_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    GATE_DRIVER_ERROR_FLAGS_LIST(X)
    #undef X
};

inline const char* to_string(GateDriverErrorFlags e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case GateDriverErrorFlags::NAME: return #NAME;
        GATE_DRIVER_ERROR_FLAGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef GATE_DRIVER_ERROR_FLAGS_LIST
/// @}

/// @name ADC Status Flags
/// @{
///--------------------------------------
//  ADC Status Flags
//--------------------------------------
/**
 * @brief Bit flags reported via parameter ADC_STATUS_FLAGS.
 *
 * NUMBER | NAME         | DESCRIPTION
 * ------ | ------------ | -----------
 * 0x00000001 | I0_CLIPPED   | No description provided.
 * 0x00000002 | I1_CLIPPED   | No description provided.
 * 0x00000004 | I2_CLIPPED   | No description provided.
 * 0x00000008 | I3_CLIPPED   | No description provided.
 * 0x00000010 | U0_CLIPPED   | No description provided.
 * 0x00000020 | U1_CLIPPED   | No description provided.
 * 0x00000040 | U2_CLIPPED   | No description provided.
 * 0x00000080 | U3_CLIPPED   | No description provided.
 * 0x00000100 | AIN0_CLIPPED | No description provided.
 * 0x00000200 | AIN1_CLIPPED | No description provided.
 * 0x00000400 | AIN2_CLIPPED | No description provided.
 * 0x00000800 | AIN3_CLIPPED | No description provided.
 * 0x00001000 | VM_CLIPPED   | No description provided.
 * 0x00002000 | TEMP_CLIPPED | No description provided.
 */
#define ADC_STATUS_FLAGS_LIST(X) \
    X(I0_CLIPPED,   0x00000001, /*!< No description provided. */) \
    X(I1_CLIPPED,   0x00000002, /*!< No description provided. */) \
    X(I2_CLIPPED,   0x00000004, /*!< No description provided. */) \
    X(I3_CLIPPED,   0x00000008, /*!< No description provided. */) \
    X(U0_CLIPPED,   0x00000010, /*!< No description provided. */) \
    X(U1_CLIPPED,   0x00000020, /*!< No description provided. */) \
    X(U2_CLIPPED,   0x00000040, /*!< No description provided. */) \
    X(U3_CLIPPED,   0x00000080, /*!< No description provided. */) \
    X(AIN0_CLIPPED, 0x00000100, /*!< No description provided. */) \
    X(AIN1_CLIPPED, 0x00000200, /*!< No description provided. */) \
    X(AIN2_CLIPPED, 0x00000400, /*!< No description provided. */) \
    X(AIN3_CLIPPED, 0x00000800, /*!< No description provided. */) \
    X(VM_CLIPPED,   0x00001000, /*!< No description provided. */) \
    X(TEMP_CLIPPED, 0x00002000, /*!< No description provided. */)

enum class AdcStatusFlags : uint32_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    ADC_STATUS_FLAGS_LIST(X)
    #undef X
};

inline const char* to_string(AdcStatusFlags e) {
    switch(e) {
        #define X(NAME, VALUE, DOC) case AdcStatusFlags::NAME: return #NAME;
        ADC_STATUS_FLAGS_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef ADC_STATUS_FLAGS_LIST
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//                              ██████╗  █████╗ ██████╗  █████╗ ███╗   ███╗███████╗                                 //
//                              ██╔══██╗██╔══██╗██╔══██╗██╔══██╗████╗ ████║██╔════╝                                 //
//                              ██████╔╝███████║██████╔╝███████║██╔████╔██║███████╗                                 //
//                              ██╔═══╝ ██╔══██║██╔══██╗██╔══██║██║╚██╔╝██║╚════██║                                 //
//                              ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║███████║                                 //
//                              ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝                                 //
//                                                                                                                  //
//==================================================================================================================//
//                                                  PARAMETERS                                                      //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum class Parameters : uint16_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,

    // Gate driver - parameters and configurations
    GATE_DRIVER_LIST(X)
    
    // Gate driver - Overcurrent protection settings
    OVERCURRENT_PROTECTION_LIST(X)

    // Gate driver - Undervoltage (UVLO) protection settings
    UNDERVOLTAGE_PROTECTION_LIST(X)

    // Gate driver - Gate-to-source (VGS) short-circuit protection settings
    VGS_SHORT_PROTECTION_LIST(X)
    
    // Motor configuration and drive settings
    MOTOR_CONFIG_LIST(X)
    
    // ADC configuration for motor current measurement
    ADC_CONFIG_LIST(X)
    
    // Feedback sensors (ABN, Hall, SPI encoders)
    FEEDBACK_SENSOR_CONFIG_LIST(X)
    
    // Torque & flux control loops
    TORQUE_FLUX_CONTROL_LIST(X)
    
    // Velocity control & ramping
    VELOCITY_CONTROL_LIST(X)
    
    // Position control & ramping
    POSITION_CONTROL_LIST(X)
    
    // Ramper stop conditions & reference switch
    RAMPER_STOP_CONFIG_LIST(X)
    
    // Biquad filter parameters
    BIQUAD_FILTER_LIST(X)
    
    // Fault-handling / retry behavior
    FAULT_HANDLING_LIST(X)
    
    // I²t monitoring
    IIT_MONITOR_LIST(X)
    
    // Temperature protection
    TEMPERATURE_PROTECTION_LIST(X)
    
    // Heartbeat monitoring (ids 3/4) and hibernation/wake-up (ids 10/11) are GLOBAL bank-0
    // parameters (SGP/GGP), not axis parameters. Listing them here aliased them onto
    // MOTOR_PWM_FREQUENCY / COMMUTATION_MODE (3/4) and two other axis ids, so a SAP of
    // "HEARTBEAT_MONITORING_TIMEOUT" wrote the commutation mode. Use GlobalParamBank0.

    // Brake chopper settings
    BRAKE_CHOPPER_LIST(X)
    
    // Mechanical brake parameters
    MECHANICAL_BRAKE_LIST(X)
    
    // Automatic homing / reference search
    REFERENCE_SEARCH_LIST(X)
    
    // STEP/DIR interface
    STEP_DIR_LIST(X)
    
    // System status and supply monitoring
    SYSTEM_STATUS_SUPPLY_LIST(X)

    // Internal measurement parameters
    INTERNAL_MEASUREMENT_LIST(X)

    // Combined diagnostic values
    COMBINED_DIAGNOSTIC_VALUES_LIST(X)
    
    // Error flags and status monitoring
    ERRORS_AND_FLAGS_LIST(X)

    #undef X
};

/**
 * @brief Convert Parameters enum to string representation
 * 
 * This function provides human-readable names for parameter IDs by looking up
 * the parameter value using integer comparisons covering all defined parameters.
 * 
 * @param param The parameter ID to convert
 * @return String representation of the parameter (e.g., "MOTOR_TYPE", "MAX_TORQUE")
 * 
 * @note This function uses integer comparison instead of switch to handle duplicate values
 */
inline const char* to_string(Parameters param) {
    uint16_t value = static_cast<uint16_t>(param);
    
    // Use X-macro to generate if-else chain for all parameters
    #define X(NAME, VALUE, DOC) if (value == static_cast<uint16_t>(Parameters::NAME)) return #NAME;
    GATE_DRIVER_LIST(X)
    OVERCURRENT_PROTECTION_LIST(X)
    UNDERVOLTAGE_PROTECTION_LIST(X)
    VGS_SHORT_PROTECTION_LIST(X)
    MOTOR_CONFIG_LIST(X)
    ADC_CONFIG_LIST(X)
    FEEDBACK_SENSOR_CONFIG_LIST(X)
    TORQUE_FLUX_CONTROL_LIST(X)
    VELOCITY_CONTROL_LIST(X)
    POSITION_CONTROL_LIST(X)
    RAMPER_STOP_CONFIG_LIST(X)
    BIQUAD_FILTER_LIST(X)
    FAULT_HANDLING_LIST(X)
    IIT_MONITOR_LIST(X)
    TEMPERATURE_PROTECTION_LIST(X)
    BRAKE_CHOPPER_LIST(X)
    MECHANICAL_BRAKE_LIST(X)
    REFERENCE_SEARCH_LIST(X)
    STEP_DIR_LIST(X)
    SYSTEM_STATUS_SUPPLY_LIST(X)
    INTERNAL_MEASUREMENT_LIST(X)
    COMBINED_DIAGNOSTIC_VALUES_LIST(X)
    ERRORS_AND_FLAGS_LIST(X)
    #undef X
    
    return "UNKNOWN_PARAM";
}

namespace detail {
/// Every axis parameter number in ::Parameters, in list order (compile-time bank check input).
inline constexpr std::uint16_t kAxisParameterIds[] = {
    #define X(NAME, VALUE, DOC) static_cast<std::uint16_t>(VALUE),
    GATE_DRIVER_LIST(X)
    OVERCURRENT_PROTECTION_LIST(X)
    UNDERVOLTAGE_PROTECTION_LIST(X)
    VGS_SHORT_PROTECTION_LIST(X)
    MOTOR_CONFIG_LIST(X)
    ADC_CONFIG_LIST(X)
    FEEDBACK_SENSOR_CONFIG_LIST(X)
    TORQUE_FLUX_CONTROL_LIST(X)
    VELOCITY_CONTROL_LIST(X)
    POSITION_CONTROL_LIST(X)
    RAMPER_STOP_CONFIG_LIST(X)
    BIQUAD_FILTER_LIST(X)
    FAULT_HANDLING_LIST(X)
    IIT_MONITOR_LIST(X)
    TEMPERATURE_PROTECTION_LIST(X)
    BRAKE_CHOPPER_LIST(X)
    MECHANICAL_BRAKE_LIST(X)
    REFERENCE_SEARCH_LIST(X)
    STEP_DIR_LIST(X)
    SYSTEM_STATUS_SUPPLY_LIST(X)
    INTERNAL_MEASUREMENT_LIST(X)
    COMBINED_DIAGNOSTIC_VALUES_LIST(X)
    ERRORS_AND_FLAGS_LIST(X)
    #undef X
};

/// True when no two entries of kAxisParameterIds share a parameter number.
constexpr bool axisParameterIdsUnique() noexcept {
    constexpr std::size_t n = sizeof(kAxisParameterIds) / sizeof(kAxisParameterIds[0]);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (kAxisParameterIds[i] == kAxisParameterIds[j]) {
                return false;
            }
        }
    }
    return true;
}
} // namespace detail

// A duplicate number means a list that does not belong to the axis bank was added to
// ::Parameters (the heartbeat/hibernation lists did exactly that), or a transcription error.
static_assert(detail::axisParameterIdsUnique(),
              "tmcl::Parameters contains duplicate axis parameter numbers; global (SGP/GGP) "
              "parameters belong in GlobalParamBank0/2/3, not in Parameters");

/* ───────────── Clean-up (optional but tidy) ───────────── */
// Undefine all parameter-section X-macros to avoid polluting the global namespace.
#undef GATE_DRIVER_LIST
#undef OVERCURRENT_PROTECTION_LIST
#undef UNDERVOLTAGE_PROTECTION_LIST
#undef VGS_SHORT_PROTECTION_LIST
#undef MOTOR_CONFIG_LIST
#undef ADC_CONFIG_LIST
#undef FEEDBACK_SENSOR_CONFIG_LIST
#undef TORQUE_FLUX_CONTROL_LIST
#undef VELOCITY_CONTROL_LIST
#undef POSITION_CONTROL_LIST
#undef RAMPER_STOP_CONFIG_LIST
#undef BIQUAD_FILTER_LIST
#undef FAULT_HANDLING_LIST
#undef IIT_MONITOR_LIST
#undef TEMPERATURE_PROTECTION_LIST
#undef HEARTBEAT_MONITORING_LIST
#undef BRAKE_CHOPPER_LIST
#undef MECHANICAL_BRAKE_LIST
#undef REFERENCE_SEARCH_LIST
#undef STEP_DIR_LIST
#undef HIBERNATION_WAKEUP_LIST
#undef SYSTEM_STATUS_SUPPLY_LIST
#undef INTERNAL_MEASUREMENT_LIST
#undef COMBINED_DIAGNOSTIC_VALUES_LIST
#undef ERRORS_AND_FLAGS_LIST
#undef FLAG

// clang-format on
} // namespace tmc9660::tmcl
