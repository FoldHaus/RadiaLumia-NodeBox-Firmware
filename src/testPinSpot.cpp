#include <avr/pgmspace.h>
#include "testPinSpot.h"

const uint16_t etable[1<<12] PROGMEM = {
     0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
     3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
     3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
     4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,
     4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
     5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,
     5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,
     6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
     7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,
     8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,
     9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,
    10,  10,  10,  10,  10,  10,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,
    12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,
    13,  13,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,
    15,  15,  15,  15,  15,  15,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,
    17,  17,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  20,  20,  20,  20,  20,  20,  20,  20,  20,
    20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  20,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  21,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  22,  23,  23,  23,
    23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  23,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  24,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  26,  26,  26,  26,  26,
    26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,
    29,  29,  29,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,
    33,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  38,  38,  38,  38,  38,  38,  38,
    38,  38,  38,  38,  38,  38,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  43,  43,  43,  43,  43,  43,  43,  43,  43,
    43,  43,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  49,  49,  49,  49,  49,  49,  49,  49,
    49,  49,  50,  50,  50,  50,  50,  50,  50,  50,  50,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  53,  53,  53,  53,  53,  53,  53,  53,  53,  54,  54,  54,  54,  54,  54,  54,  54,  54,  55,  55,  55,  55,  55,  55,  55,  55,  55,  56,  56,  56,  56,  56,  56,
    56,  56,  56,  57,  57,  57,  57,  57,  57,  57,  57,  58,  58,  58,  58,  58,  58,  58,  58,  58,  59,  59,  59,  59,  59,  59,  59,  59,  60,  60,  60,  60,  60,  60,  60,  60,  61,  61,  61,  61,  61,  61,  61,  61,  62,  62,  62,  62,  62,  62,  62,  62,  63,  63,  63,  63,  63,  63,  63,  63,  64,  64,  64,  64,
    64,  64,  64,  64,  65,  65,  65,  65,  65,  65,  65,  66,  66,  66,  66,  66,  66,  66,  66,  67,  67,  67,  67,  67,  67,  67,  68,  68,  68,  68,  68,  68,  68,  68,  69,  69,  69,  69,  69,  69,  69,  70,  70,  70,  70,  70,  70,  70,  71,  71,  71,  71,  71,  71,  71,  72,  72,  72,  72,  72,  72,  73,  73,  73,
    73,  73,  73,  73,  74,  74,  74,  74,  74,  74,  74,  75,  75,  75,  75,  75,  75,  76,  76,  76,  76,  76,  76,  76,  77,  77,  77,  77,  77,  77,  78,  78,  78,  78,  78,  78,  78,  79,  79,  79,  79,  79,  79,  80,  80,  80,  80,  80,  80,  81,  81,  81,  81,  81,  81,  82,  82,  82,  82,  82,  82,  83,  83,  83,
    83,  83,  83,  84,  84,  84,  84,  84,  84,  85,  85,  85,  85,  85,  85,  86,  86,  86,  86,  86,  87,  87,  87,  87,  87,  87,  88,  88,  88,  88,  88,  88,  89,  89,  89,  89,  89,  90,  90,  90,  90,  90,  90,  91,  91,  91,  91,  91,  92,  92,  92,  92,  92,  93,  93,  93,  93,  93,  93,  94,  94,  94,  94,  94,
    95,  95,  95,  95,  95,  96,  96,  96,  96,  96,  97,  97,  97,  97,  97,  98,  98,  98,  98,  98,  99,  99,  99,  99,  99, 100, 100, 100, 100, 100, 101, 101, 101, 101, 101, 102, 102, 102, 102, 102, 103, 103, 103, 103, 103, 104, 104, 104, 104, 105, 105, 105, 105, 105, 106, 106, 106, 106, 106, 107, 107, 107, 107, 108,
   108, 108, 108, 108, 109, 109, 109, 109, 110, 110, 110, 110, 110, 111, 111, 111, 111, 112, 112, 112, 112, 112, 113, 113, 113, 113, 114, 114, 114, 114, 115, 115, 115, 115, 115, 116, 116, 116, 116, 117, 117, 117, 117, 118, 118, 118, 118, 119, 119, 119, 119, 120, 120, 120, 120, 120, 121, 121, 121, 121, 122, 122, 122, 122,
   123, 123, 123, 123, 124, 124, 124, 124, 125, 125, 125, 125, 126, 126, 126, 126, 127, 127, 127, 128, 128, 128, 128, 129, 129, 129, 129, 130, 130, 130, 130, 131, 131, 131, 131, 132, 132, 132, 133, 133, 133, 133, 134, 134, 134, 134, 135, 135, 135, 136, 136, 136, 136, 137, 137, 137, 137, 138, 138, 138, 139, 139, 139, 139,
   140, 140, 140, 141, 141, 141, 141, 142, 142, 142, 143, 143, 143, 143, 144, 144, 144, 145, 145, 145, 146, 146, 146, 146, 147, 147, 147, 148, 148, 148, 149, 149, 149, 149, 150, 150, 150, 151, 151, 151, 152, 152, 152, 152, 153, 153, 153, 154, 154, 154, 155, 155, 155, 156, 156, 156, 157, 157, 157, 158, 158, 158, 158, 159,
   159, 159, 160, 160, 160, 161, 161, 161, 162, 162, 162, 163, 163, 163, 164, 164, 164, 165, 165, 165, 166, 166, 166, 167, 167, 167, 168, 168, 168, 169, 169, 169, 170, 170, 171, 171, 171, 172, 172, 172, 173, 173, 173, 174, 174, 174, 175, 175, 175, 176, 176, 176, 177, 177, 178, 178, 178, 179, 179, 179, 180, 180, 180, 181,
   181, 182, 182, 182, 183, 183, 183, 184, 184, 185, 185, 185, 186, 186, 186, 187, 187, 188, 188, 188, 189, 189, 190, 190, 190, 191, 191, 191, 192, 192, 193, 193, 193, 194, 194, 195, 195, 195, 196, 196, 197, 197, 197, 198, 198, 199, 199, 199, 200, 200, 201, 201, 201, 202, 202, 203, 203, 203, 204, 204, 205, 205, 206, 206,
   206, 207, 207, 208, 208, 208, 209, 209, 210, 210, 211, 211, 211, 212, 212, 213, 213, 214, 214, 214, 215, 215, 216, 216, 217, 217, 218, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 222, 223, 223, 224, 224, 225, 225, 226, 226, 227, 227, 228, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, 233, 234, 234, 235,
   235, 236, 236, 236, 237, 237, 238, 238, 239, 239, 240, 240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 247, 247, 248, 248, 249, 249, 250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255, 256, 256, 257, 258, 258, 259, 259, 260, 260, 261, 261, 262, 262, 263, 263, 264, 264, 265, 265, 266, 267, 267,
   268, 268, 269, 269, 270, 270, 271, 271, 272, 273, 273, 274, 274, 275, 275, 276, 276, 277, 278, 278, 279, 279, 280, 280, 281, 282, 282, 283, 283, 284, 284, 285, 286, 286, 287, 287, 288, 289, 289, 290, 290, 291, 291, 292, 293, 293, 294, 294, 295, 296, 296, 297, 297, 298, 299, 299, 300, 301, 301, 302, 302, 303, 304, 304,
   305, 305, 306, 307, 307, 308, 309, 309, 310, 310, 311, 312, 312, 313, 314, 314, 315, 316, 316, 317, 317, 318, 319, 319, 320, 321, 321, 322, 323, 323, 324, 325, 325, 326, 327, 327, 328, 329, 329, 330, 331, 331, 332, 333, 333, 334, 335, 335, 336, 337, 337, 338, 339, 339, 340, 341, 342, 342, 343, 344, 344, 345, 346, 346,
   347, 348, 349, 349, 350, 351, 351, 352, 353, 354, 354, 355, 356, 356, 357, 358, 359, 359, 360, 361, 362, 362, 363, 364, 364, 365, 366, 367, 367, 368, 369, 370, 370, 371, 372, 373, 373, 374, 375, 376, 377, 377, 378, 379, 380, 380, 381, 382, 383, 383, 384, 385, 386, 387, 387, 388, 389, 390, 391, 391, 392, 393, 394, 395,
   395, 396, 397, 398, 399, 399, 400, 401, 402, 403, 403, 404, 405, 406, 407, 408, 408, 409, 410, 411, 412, 413, 413, 414, 415, 416, 417, 418, 418, 419, 420, 421, 422, 423, 424, 424, 425, 426, 427, 428, 429, 430, 431, 431, 432, 433, 434, 435, 436, 437, 438, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 447, 448, 449,
   450, 451, 452, 453, 454, 455, 456, 457, 458, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 505, 506, 508, 509, 510, 511, 512,
   513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 542, 543, 544, 545, 546, 547, 548, 549, 550, 552, 553, 554, 555, 556, 557, 558, 559, 561, 562, 563, 564, 565, 566, 567, 569, 570, 571, 572, 573, 574, 576, 577, 578, 579, 580, 581, 583,
   584, 585, 586, 587, 589, 590, 591, 592, 593, 595, 596, 597, 598, 599, 601, 602, 603, 604, 606, 607, 608, 609, 611, 612, 613, 614, 616, 617, 618, 619, 621, 622, 623, 624, 626, 627, 628, 629, 631, 632, 633, 635, 636, 637, 638, 640, 641, 642, 644, 645, 646, 648, 649, 650, 652, 653, 654, 656, 657, 658, 660, 661, 662, 664,
   665, 666, 668, 669, 670, 672, 673, 674, 676, 677, 679, 680, 681, 683, 684, 685, 687, 688, 690, 691, 692, 694, 695, 697, 698, 700, 701, 702, 704, 705, 707, 708, 710, 711, 712, 714, 715, 717, 718, 720, 721, 723, 724, 726, 727, 729, 730, 732, 733, 734, 736, 737, 739, 740, 742, 743, 745, 747, 748, 750, 751, 753, 754, 756,
   757, 759, 760, 762, 763, 765, 766, 768, 770, 771, 773, 774, 776, 777, 779, 781, 782, 784, 785, 787, 789, 790, 792, 793, 795, 797, 798, 800, 802, 803, 805, 806, 808, 810, 811, 813, 815, 816, 818, 820, 821, 823, 825, 826, 828, 830, 831, 833, 835, 836, 838, 840, 842, 843, 845, 847, 848, 850, 852, 854, 855, 857, 859, 861,
   862, 864, 866, 868, 869, 871, 873, 875, 876, 878, 880, 882, 884, 885, 887, 889, 891, 893, 894, 896, 898, 900, 902, 904, 905, 907, 909, 911, 913, 915, 917, 918, 920, 922, 924, 926, 928, 930, 932, 933, 935, 937, 939, 941, 943, 945, 947, 949, 951, 953, 955, 956, 958, 960, 962, 964, 966, 968, 970, 972, 974, 976, 978, 980,
   982, 984, 986, 988, 990, 992, 994, 996, 998,1000,1002,1004,1006,1008,1010,1012,1014,1017,1019,1021,1023,1025,1027,1029,1031,1033,1035,1037,1040,1042,1044,1046,1048,1050,1052,1054,1057,1059,1061,1063,1065,1067,1069,1072,1074,1076,1078,1080,1083,1085,1087,1089,1091,1094,1096,1098,1100,1103,1105,1107,1109,1112,1114,1116,
  1118,1121,1123,1125,1127,1130,1132,1134,1137,1139,1141,1144,1146,1148,1151,1153,1155,1158,1160,1162,1165,1167,1169,1172,1174,1177,1179,1181,1184,1186,1189,1191,1193,1196,1198,1201,1203,1206,1208,1211,1213,1215,1218,1220,1223,1225,1228,1230,1233,1235,1238,1240,1243,1245,1248,1251,1253,1256,1258,1261,1263,1266,1268,1271,
  1274,1276,1279,1281,1284,1287,1289,1292,1294,1297,1300,1302,1305,1308,1310,1313,1316,1318,1321,1324,1326,1329,1332,1335,1337,1340,1343,1345,1348,1351,1354,1356,1359,1362,1365,1367,1370,1373,1376,1379,1381,1384,1387,1390,1393,1396,1398,1401,1404,1407,1410,1413,1415,1418,1421,1424,1427,1430,1433,1436,1439,1442,1445,1447,
  1450,1453,1456,1459,1462,1465,1468,1471,1474,1477,1480,1483,1486,1489,1492,1495,1498,1501,1504,1507,1511,1514,1517,1520,1523,1526,1529,1532,1535,1538,1542,1545,1548,1551,1554,1557,1560,1564,1567,1570,1573,1576,1580,1583,1586,1589,1592,1596,1599,1602,1605,1609,1612,1615,1619,1622,1625,1628,1632,1635,1638,1642,1645,1648,
  1652,1655,1658,1662,1665,1669,1672,1675,1679,1682,1686,1689,1693,1696,1699,1703,1706,1710,1713,1717,1720,1724,1727,1731,1734,1738,1741,1745,1748,1752,1756,1759,1763,1766,1770,1773,1777,1781,1784,1788,1792,1795,1799,1803,1806,1810,1814,1817,1821,1825,1828,1832,1836,1840,1843,1847,1851,1855,1858,1862,1866,1870,1873,1877,
  1881,1885,1889,1893,1896,1900,1904,1908,1912,1916,1920,1924,1927,1931,1935,1939,1943,1947,1951,1955,1959,1963,1967,1971,1975,1979,1983,1987,1991,1995,1999,2003,2007,2011,2016,2020,2024,2028,2032,2036,2040,2044,2049,2053,2057,2061,2065,2069,2074,2078,2082,2086,2091,2095,2099,2103,2108,2112,2116,2121,2125,2129,2134,2138,
  2142,2147,2151,2155,2160,2164,2168,2173,2177,2182,2186,2191,2195,2200,2204,2208,2213,2217,2222,2226,2231,2236,2240,2245,2249,2254,2258,2263,2268,2272,2277,2281,2286,2291,2295,2300,2305,2309,2314,2319,2324,2328,2333,2338,2342,2347,2352,2357,2362,2366,2371,2376,2381,2386,2391,2395,2400,2405,2410,2415,2420,2425,2430,2435,
  2440,2445,2450,2454,2459,2464,2469,2475,2480,2485,2490,2495,2500,2505,2510,2515,2520,2525,2530,2536,2541,2546,2551,2556,2561,2567,2572,2577,2582,2588,2593,2598,2603,2609,2614,2619,2625,2630,2635,2641,2646,2651,2657,2662,2668,2673,2679,2684,2689,2695,2700,2706,2711,2717,2722,2728,2733,2739,2745,2750,2756,2761,2767,2773,
  2778,2784,2790,2795,2801,2807,2812,2818,2824,2829,2835,2841,2847,2853,2858,2864,2870,2876,2882,2888,2893,2899,2905,2911,2917,2923,2929,2935,2941,2947,2953,2959,2965,2971,2977,2983,2989,2995,3001,3007,3013,3020,3026,3032,3038,3044,3050,3057,3063,3069,3075,3081,3088,3094,3100,3107,3113,3119,3126,3132,3138,3145,3151,3158,
  3164,3170,3177,3183,3190,3196,3203,3209,3216,3222,3229,3235,3242,3249,3255,3262,3268,3275,3282,3288,3295,3302,3309,3315,3322,3329,3335,3342,3349,3356,3363,3370,3376,3383,3390,3397,3404,3411,3418,3425,3432,3439,3446,3453,3460,3467,3474,3481,3488,3495,3502,3509,3516,3524,3531,3538,3545,3552,3559,3567,3574,3581,3589,3596,
  3603,3610,3618,3625,3633,3640,3647,3655,3662,3670,3677,3685,3692,3700,3707,3715,3722,3730,3737,3745,3753,3760,3768,3775,3783,3791,3799,3806,3814,3822,3830,3837,3845,3853,3861,3869,3876,3884,3892,3900,3908,3916,3924,3932,3940,3948,3956,3964,3972,3980,3988,3996,4005,4013,4021,4029,4037,4045,4054,4062,4070,4078,4087,4095,

};

uint16_t curvePS(uint16_t i) {
 // If the compiler could build the above table for us, we could have "simply":
 //return !i ? 0 : round(exp(log(MAXOUTPUT) * i / MAXINPUT));
 
 // Need to read the table in a special way!
 return pgm_read_word(&etable[i]);
}