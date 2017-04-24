/*
 * CFile1.c
 *
 * Created: 4/1/2017 7:28:05 PM
 *  Author: aman1
 */ 

// default status parameters always written to flash during programming boot code. 7 bytes
const unsigned char params[]  __attribute__((section (".status"))) = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};


