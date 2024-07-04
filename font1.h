/* 
 * File:   font1.h
 * Author: Hennie Kotze firmware@autonomech.co.za
 *
 * Created on 03 April 2019,2:38 PM
 */

#ifndef FONT1H
#define FONT1H

const uint16_t charSPACE[2] PROGMEM = {1, 0x0000};
const uint16_t charPERIOD[2] PROGMEM = {1, 0x2611};
const uint16_t charCOLON[3] PROGMEM = {2,
    0x2111,
    0x2511};
const uint16_t charQUOTE[3] PROGMEM = {2,
    0x1012,
    0x3012};
const uint16_t charCOMMA[3] PROGMEM = {2,
    0x2412,
    0x1611};
//----;
const uint16_t charSEMICOLON[4] PROGMEM = {3,
    0x2111,
    0x2412,
    0x1611};
//----$
const uint16_t charDOLLAR[7] PROGMEM = {6,
    0x2017,
    0x1141,
    0x0211,
    0x1331,
    0x4411,
    0x0541};
//----PLUS
const uint16_t charPLUS[3] PROGMEM = {2,
    0x0351,
    0x2115};
//----=
const uint16_t charEQ[3] PROGMEM = {2,
    0x0251,
    0x0451};
//----DASH/MINUS
const uint16_t charDASH[2] PROGMEM = {1,
    0x0351};
//----!
const uint16_t charEXCL[3] PROGMEM = {2,
    0x2015,
    0x2611};
//----1
const uint16_t char1[5] PROGMEM = {4,
    0x0211,
    0x1111,
    0x2017,
    0x0651};
//----2
const uint16_t char2[8] PROGMEM = {7,
    0x0111,
    0x1031,
    0x4112,
    0x3311,
    0x2411,
    0x1511,
    0x0651};
//----3
const uint16_t char3[8] PROGMEM = {7,
    0x0111,
    0x1031,
    0x4112,
    0x2321,
    0x4412,
    0x1631,
    0x0511};
//----4
const uint16_t char4[5] PROGMEM = {4,
    0x3017,
    0x2111,
    0x1211,
    0x0351};
//----5
const uint16_t char5[6] PROGMEM = {5,
    0x0051,
    0x0112,
    0x0341,
    0x4412,
    0x0641};
//----6
const uint16_t char6[6] PROGMEM = {5,
    0x1031,
    0x0115,
    0x1331,
    0x4412,
    0x1631};
//----7

const uint16_t char7[6] PROGMEM = {5,
    0x0051,
    0x4111,
    0x3211,
    0x2311,
    0x1413};
//----8

const uint16_t char8[8] PROGMEM = {7,
    0x1031,
    0x0112,
    0x4112,
    0x1331,
    0x0412,
    0x4412,
    0x1631};
//----9
const uint16_t char9[6] PROGMEM = {5,
    0x1031,
    0x0112,
    0x4115,
    0x1331,
    0x1631};
//----0
const uint16_t char0[8] PROGMEM = {7,
    0x1031,
    0x0115,
    0x4115,
    0x1631,
    0x3211,
    0x2311,
    0x1411};
//----*
const uint16_t charASTERISK[7] PROGMEM = {6,
    0x0111,
    0x4111,
    0x0511,
    0x4511,
    0x1233,
    0x2017};
//----a
const uint16_t chara[6] PROGMEM = {5,
    0x1231,
    0x1431,
    0x1631,
    0x0511,
    0x4314};
//----A
const uint16_t charA[5] PROGMEM = {4,
    0x0116,
    0x4116,
    0x1031,
    0x1331};
//----b
const uint16_t charb[5] PROGMEM = {4,
    0x0017,
    0x1231,
    0x1631,
    0x4313};
//----B
const uint16_t charB[7] PROGMEM = {6,
    0x0017,
    0x1031,
    0x1331,
    0x1631,
    0x4112,
    0x4412};
//----c
const uint16_t charc[6] PROGMEM = {5,
    0x0313,
    0x1231,
    0x1631,
    0x4311,
    0x4511};
//----C
const uint16_t charC[6] PROGMEM = {5,
    0x0115,
    0x1031,
    0x1631,
    0x4111,
    0x4511};
//----d
const uint16_t chard[5] PROGMEM = {4,
    0x4017,
    0x1231,
    0x1631,
    0x0313};
//----D
const uint16_t charD[5] PROGMEM = {4,
    0x0017,
    0x1031,
    0x1631,
    0x4115};
//----e
const uint16_t chare[6] PROGMEM = {5,
    0x0313,
    0x1231,
    0x1641,
    0x4311,
    0x1441};
//----E
const uint16_t charE[5] PROGMEM = {4,
    0x0017,
    0x1041,
    0x1641,
    0x1331};
//----f
const uint16_t charf[4] PROGMEM = {3,
    0x1116,
    0x2021,
    0x0351};
//----F
const uint16_t charF[4] PROGMEM = {3,
    0x0017,
    0x1041,
    0x1331};
//----g
const uint16_t charg[7] PROGMEM = {6,
    0x4214,
    0x1231,
    0x0311,
    0x1431,
    0x1631,};
//----G
const uint16_t charG[7] PROGMEM = {6,
    0x0115,
    0x1031,
    0x1631,
    0x4111,
    0x4412,
    0x2331};
//----h
const uint16_t charh[4] PROGMEM = {3,
    0x0017,
    0x4314,
    0x1231};
//----H
const uint16_t charH[4] PROGMEM = {3,
    0x0017,
    0x4017,
    0x1331};
//----i
const uint16_t chari[5] PROGMEM = {4,
    0x0651,
    0x2011,
    0x0221,
    0x2215};
//----I
const uint16_t charI[4] PROGMEM = {3,
    0x2017,
    0x0051,
    0x0651};
//----j
const uint16_t charj[6] PROGMEM = {5,
    0x4011,
    0x2231,
    0x4313,
    0x0511,
    0x1631};
//----J
const uint16_t charJ[5] PROGMEM = {4,
    0x0051,
    0x4115,
    0x1631,
    0x0511};
//----k
const uint16_t chark[6] PROGMEM = {5,
    0x0017,
    0x1321,
    0x3211,
    0x3411,
    0x4512};
//----K
const uint16_t charK[7] PROGMEM = {6,
    0x0017,
    0x1321,
    0x3211,
    0x3411,
    0x4012,
    0x4512};
//----l
const uint16_t charl[3] PROGMEM = {2,
    0x0016,
    0x1631};
//----L
const uint16_t charL[3] PROGMEM = {2,
    0x0017,
    0x1641};
//----m
const uint16_t charm[6] PROGMEM = {5,
    0x0215,
    0x1211,
    0x2312,
    0x3211,
    0x4314};
//----M
const uint16_t charM[6] PROGMEM = {5,
    0x0017,
    0x4017,
    0x1111,
    0x2211,
    0x3111};
//----n
const uint16_t charn[4] PROGMEM = {3,
    0x0215,
    0x1231,
    0x4314};
//----N
const uint16_t charN[6] PROGMEM = {5,
    0x0017,
    0x4017,
    0x1211,
    0x2311,
    0x3411};
//----o
const uint16_t charo[5] PROGMEM = {4,
    0x0313,
    0x4313,
    0x1231,
    0x1631};
//----O
const uint16_t charO[5] PROGMEM = {4,
    0x0115,
    0x4115,
    0x1031,
    0x1631};
//----p
const uint16_t charp[5] PROGMEM = {4,
    0x0215,
    0x1231,
    0x1431,
    0x4311};
//----P
const uint16_t charP[5] PROGMEM = {4,
    0x0017,
    0x1031,
    0x1331,
    0x4112};
//----q
const uint16_t charq[5] PROGMEM = {4,
    0x0311,
    0x1231,
    0x1431,
    0x4215};
//----Q
const uint16_t charQ[8] PROGMEM = {7,
    0x0115,
    0x4114,
    0x1031,
    0x2411,
    0x3511,
    0x4611,
    0x1621};
//----r
const uint16_t charr[5] PROGMEM = {4,
    0x0215,
    0x1311,
    0x2221,
    0x4311};
//----R
const uint16_t charR[6] PROGMEM = {5,
    0x0017,
    0x1031,
    0x1331,
    0x4112,
    0x4413};
//----s
const uint16_t chars[6] PROGMEM = {5,
    0x1241,
    0x0311,
    0x1441,
    0x4511,
    0x0641};
//----S
const uint16_t charS[6] PROGMEM = {5,
    0x1041,
    0x0112,
    0x1331,
    0x4412,
    0x0641};
//----t
const uint16_t chart[5] PROGMEM = {4,
    0x1016,
    0x0241,
    0x2621,
    0x4511};
//----T
const uint16_t charT[3] PROGMEM = {2,
    0x0051,
    0x2116};
//----u
const uint16_t charu[4] PROGMEM = {3,
    0x0214,
    0x4214,
    0x1641};
//----U
const uint16_t charU[4] PROGMEM = {3,
    0x0016,
    0x4016,
    0x1631};
//----v
const uint16_t charv[6] PROGMEM = {5,
    0x0213,
    0x4213,
    0x1511,
    0x3511,
    0x2611};
//----V
const uint16_t charV[6] PROGMEM = {5,
    0x0015,
    0x4015,
    0x1511,
    0x2611,
    0x3511};
//----w
const uint16_t charw[6] PROGMEM = {5,
    0x0214,
    0x4214,
    0x1611,
    0x2511,
    0x3611};
//----W
const uint16_t charW[6] PROGMEM = {5,
    0x0017,
    0x4017,
    0x1511,
    0x2411,
    0x3511};
//----x
const uint16_t charx[10] PROGMEM = {9,
    0x0211,
    0x4211,
    0x1311,
    0x3311,
    0x2411,
    0x1511,
    0x3511,
    0x0611,
    0x4611};
//----X
const uint16_t charX[10] PROGMEM = {9,
    0x0012,
    0x4012,
    0x0512,
    0x4512,
    0x1211,
    0x3211,
    0x2311,
    0x1411,
    0x3411};
//----y
const uint16_t chary[5] PROGMEM = {4,
    0x0212,
    0x1431,
    0x4214,
    0x1631};
//----Y
const uint16_t charY[6] PROGMEM = {5,
    0x0012,
    0x4012,
    0x2314,
    0x1211,
    0x3211};
//----z
const uint16_t charz[6] PROGMEM = {5,
    0x0251,
    0x3311,
    0x2411,
    0x1511,
    0x0651};
//----Z
const uint16_t charZ[8] PROGMEM = {7,
    0x0051,
    0x0651,
    0x4111,
    0x3211,
    0x2311,
    0x1411,
    0x0511};



//==============================================================

const uint16_t ** get_char(char c) {
    switch (c) {
        case '1':
            return (const uint16_t **) &char1;
        case '2':
            return (const uint16_t **) &char2;
        case '3':
            return (const uint16_t **) &char3;
        case '4':
            return (const uint16_t **) &char4;
        case '5':
            return (const uint16_t **) &char5;
        case '6':
            return (const uint16_t **) &char6;
        case '7':
            return (const uint16_t **) &char7;
        case '8':
            return (const uint16_t **) &char8;
        case '9':
            return (const uint16_t **) &char9;
        case '0':
            return (const uint16_t **) &char0;
        case ' ':
            return (const uint16_t **) &charSPACE;
        case '!':
            return (const uint16_t **) &charEXCL;
        case '*':
            return (const uint16_t **) &charASTERISK;
        case '.':
            return (const uint16_t **) &charPERIOD;
        case ':':
            return (const uint16_t **) &charCOLON;
        case ';':
            return (const uint16_t **) &charSEMICOLON;
        case ',':
            return (const uint16_t **) &charCOMMA;
        case '-':
            return (const uint16_t **) &charDASH;
        case '+':
            return (const uint16_t **) &charPLUS;
        case '=':
            return (const uint16_t **) &charEQ;
        case '"':
            return (const uint16_t **) &charQUOTE;
        case '$':
            return (const uint16_t **) &charDOLLAR;
        case'a':
            return (const uint16_t **) &chara;
        case 'A':
            return (const uint16_t **) &charA;
        case'b':
            return (const uint16_t **) &charb;
        case 'B':
            return (const uint16_t **) &charB;
        case'c':
            return (const uint16_t **) &charc;
        case 'C':
            return (const uint16_t **) &charC;
        case'd':
            return (const uint16_t **) &chard;
        case 'D':
            return (const uint16_t **) &charD;
        case'e':
            return (const uint16_t **) &chare;
        case 'E':
            return (const uint16_t **) &charE;
        case'f':
            return (const uint16_t **) &charf;
        case 'F':
            return (const uint16_t **) &charF;
        case 'g':
            return (const uint16_t **) &charg;
        case 'G':
            return (const uint16_t **) &charG;
        case 'h':
            return (const uint16_t **) &charh;
        case 'H':
            return (const uint16_t **) &charH;
        case 'i':
            return (const uint16_t **) &chari;
        case 'I':
            return (const uint16_t **) &charI;
        case 'j':
            return (const uint16_t **) &charj;
        case 'J':
            return (const uint16_t **) &charJ;
        case 'k':
            return (const uint16_t **) &chark;
        case 'K':
            return (const uint16_t **) &charK;
        case 'l':
            return (const uint16_t **) &charl;
        case 'L':
            return (const uint16_t **) &charL;
        case 'm':
            return (const uint16_t **) &charm;
        case 'M':
            return (const uint16_t **) &charM;
        case 'n':
            return (const uint16_t **) &charn;
        case 'N':
            return (const uint16_t **) &charN;
        case 'o':
            return (const uint16_t **) &charo;
        case 'O':
            return (const uint16_t **) &charO;
        case 'p':
            return (const uint16_t **) &charp;
        case 'P':
            return (const uint16_t **) &charP;
        case 'q':
            return (const uint16_t **) &charq;
        case 'Q':
            return (const uint16_t **) &charQ;
        case 'r':
            return (const uint16_t **) &charr;
        case 'R':
            return (const uint16_t **) &charR;
        case 's':
            return (const uint16_t **) &chars;
        case 'S':
            return (const uint16_t **) &charS;
        case 't':
            return (const uint16_t **) &chart;
        case 'T':
            return (const uint16_t **) &charT;
        case 'u':
            return (const uint16_t **) &charu;
        case 'U':
            return (const uint16_t **) &charU;
        case 'v':
            return (const uint16_t **) &charv;
        case 'V':
            return (const uint16_t **) &charV;
        case 'w':
            return (const uint16_t **) &charw;
        case 'W':
            return (const uint16_t **) &charW;
        case 'x':
            return (const uint16_t **) &charx;
        case 'X':
            return (const uint16_t **) &charX;
        case 'y':
            return (const uint16_t **) &chary;
        case 'Y':
            return (const uint16_t **) &charY;
        case 'z':
            return (const uint16_t **) &charz;
        case 'Z':
            return (const uint16_t **) &charZ;

        default:
            return (const uint16_t **) &charPERIOD;
    }
}


#endif /* FONT1 */

