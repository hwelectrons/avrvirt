/* Copyright (c) 2017, Attila Laszlo Agas (http://www.hwelectrons.com)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the copyright holder nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AVRVIRT_DEBUG_H
#define AVRVIRT_DEBUG_H

//<xxxx>  20  64 --> 64
// 6    2 20  64 5   64

#ifdef AVRVIRT_DEBUG

#define REGNUM(ref) ((uint8_t)(((uint8_t *)&ref) - m_r))
#define DECODED(fmt,...) \
{\
  static const char fmt_str[] PROGMEM = fmt; \
  snprintf_P(m_dbgLine+8,AVRVIRT_DEBUG_BUFFERSIZE-8,fmt_str,__VA_ARGS__);\
}
  
#define BEFORE(fmt,...) \
{\
  static const char fmt_str[] PROGMEM = fmt; \
  snprintf_P(m_dbgLine+28,AVRVIRT_DEBUG_BUFFERSIZE-28,fmt_str,__VA_ARGS__);\
}

#define AFTER(fmt,...) \
{\
  static const char fmt_str[] PROGMEM = fmt; \
  snprintf_P(m_dbgLine+28+64+5,AVRVIRT_DEBUG_BUFFERSIZE-28-64-5,fmt_str,__VA_ARGS__);\
}

#endif //AVRVIRT_DEBUG

#endif //AVRVIRT_DEBUG_H