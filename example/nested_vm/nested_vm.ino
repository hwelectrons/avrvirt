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

/**
 * This sketch is starting 2 virtual machines in a manner where the second is nested under the first instance.
 * 
 * Class VM is subclassed into MyVM. The method ReadDataByte() is overridden to test for the address of variable
 * 'vm_nestcount'. When that address is accessed an increased value is given back. With this method a code reading
 * the value 'vm_nestcount' can detect whether it is running under virtualization and how many times it's nested.
 */

#include <avrvirt.h>
#include <stdio.h>

volatile uint8_t vm_nestcount = 0;
uint8_t m_stack[2][128]; //2 stack buffer for virtual machines

class MyVM : public avrvirt::VM
{
public:
  MyVM()
  {
    //Setting the stack pointer to top of the buffer
    //Note: vm_nestcount is 0 for the native instance and 1 for the virtualized instance
    setSP((uint16_t)&m_stack[vm_nestcount][128]);
  }

  virtual const uint8_t ReadDataByte(const uint16_t iAddress)
  {
    //VM is reading memory through this member function.
    //When the vm_nestcount is going to be read then we return an increased value 
    if (iAddress == ((uint16_t)&vm_nestcount))
    {
      return vm_nestcount + 1;
    }
    else
    {
      //For every other read we are using the default implementation
      return VM::ReadDataByte(iAddress);
    }
  }
};

//forward declaration
void test();

//Starting a VM instance is so easy :)
void startvm()
{
  MyVM vm;
  vm.setPC((uint16_t)test); //We set the PC to the address of a void function.
  vm.run(); //It will run until some special opcode is encountered. (I will use the opcode 'break' to terminate)
}

//This is where the fun happens.
void test()
{
  if (vm_nestcount == 0) //Host code
  {
    Serial.println("Hello, I'm the VM host.");
    startvm(); //start a vm which will execute this function.
  }
  else if (vm_nestcount == 1) //Guest code, level 1
  {
    Serial.println("Hello, I'm the VM guest.");
    startvm(); //start again vm which will execute this function.
    __asm__ __volatile__ 
    ("break\n\t" : :); //This will terminate the VM
  }
  else //Guest code, level 2
  {
    Serial.println("Hello, I'm a VM guest running under a VM guest :)");
    __asm__ __volatile__ 
    ("break\n\t" : :); //This will terminate the VM
  }
}

//setup
void setup()
{
  // initialize serial and wait for port to open:
  Serial.begin(57600);
}

//loop
void loop()
{
  Serial.print("Init...");
  Serial.println();
  test();
 
  delay(5000);
}
