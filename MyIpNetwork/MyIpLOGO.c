#ifndef _MYIPLOGO_C_
#define _MYIPLOGO_C_

#include "options.h"
#if ((USE_WIFI_NETWORK_LINK == 1) || (USE_ETHERNET_NETWORK_LINK == 1))
#if (NETWORK_USE_LOGO == 1)

#include "TCPIP_Stack/TCPIP.h"
#include "defines.h"
#include "MyIpData.h"
#include "MyIpLOGO.h"
#include "euler_angles.h"

//////////////////////////
// Module Variables
char MyIpfp_high_byte;
unsigned char MyIpfp_checksum;


void MyIpsio_newMsg(unsigned char);
void MyIpsio_fp_data( unsigned char inchar ) ;
void MyIpsio_fp_checksum( unsigned char inchar ) ;

void (* MyIpsio_parse ) ( unsigned char inchar ) = &MyIpsio_newMsg ;


void MyIpOnConnect_LOGO(BYTE s) {
    // Print any one-time connection annoucement text
    LoadStringSocket(s, "\r\nYou've connected to LOGO on "); // 33 chars
    LoadStringSocket(s, ID_LEAD_PILOT); // 15ish chars
    LoadStringSocket(s, "'s aircraft. More info at "); // 26 chars
    LoadStringSocket(s, ID_DIY_DRONES_URL); // 45ish chars
    LoadStringSocket(s, "\r\n"); // 2 chars
    MyIpData[s].sendPacket = TRUE; // send right away
}

void MyIpInit_LOGO(BYTE s) {
    // This gets called once for every socket we're configured to use for this module.
}

void MyIpService_LOGO(BYTE s) {
    
}

BOOL MyIpThreadSafeSendPacketCheck_LOGO(BYTE s, BOOL doClearFlag) {
    // since this data comes from, and goes to, the idle thread we
    // don't need to deal with any thread issues
    BOOL sendpacket = MyIpData[s].sendPacket;
    if (doClearFlag) {
        MyIpData[s].sendPacket = FALSE;
    }
    return sendpacket;
}

int MyIpThreadSafeReadBufferHead_LOGO(BYTE s) {
    // since this data comes from, and goes to, the idle thread we
    //  don't need to deal with any thread issues
    return MyIpData[s].buffer_head;
}

void MyIpProcessRxData_LOGO(BYTE s) {
    BYTE rxchar;
    BOOL successfulRead;

    do {
        if (eTCP == MyIpData[s].type) {
            successfulRead = TCPGet(MyIpData[s].socket, &rxchar);
        } else //if (eUDP == MyIpData[s].type)
        {
            successfulRead = UDPGet(&rxchar);
        }

        if (successfulRead)
        {
            (* MyIpsio_parse) ( rxchar ) ; // parse the input byte
        }
    } while (successfulRead);
}

char MyIphex_char_val(unsigned char inchar)
{
	if (inchar >= '0' && inchar <= '9')
	{
		return (inchar - '0') ;
	}
	else if (inchar >= 'A' && inchar <= 'F')
	{
		return (inchar - 'A' + 10) ;
	}
	return -1 ;
}
void MyIpsio_newMsg( unsigned char inchar )
{
    switch (inchar)
    {
#if ( FLIGHT_PLAN_TYPE == FP_LOGO )
    case 'L':
#else
    case 'W':
#endif
        MyIpfp_high_byte = -1 ; // -1 means we don't have the high byte yet (0-15 means we do)
        MyIpfp_checksum = 0 ;
        MyIpsio_parse = &MyIpsio_fp_data ;
        flightplan_live_begin() ;
        break;
    }

}
void MyIpsio_fp_data( unsigned char inchar )
{
	if (inchar == '*')
	{
		MyIpfp_high_byte = -1 ;
		MyIpsio_parse = &MyIpsio_fp_checksum ;
	}
	else
	{
		char hexVal = MyIphex_char_val(inchar) ;
		if (hexVal == -1)
		{
			MyIpsio_parse = &MyIpsio_newMsg ;
			return ;
		}
		else if (MyIpfp_high_byte == -1)
		{
			MyIpfp_high_byte = hexVal * 16 ;
		}
		else
		{
			flightplan_live_received_byte(MyIpfp_high_byte + hexVal) ;
			MyIpfp_high_byte = -1 ;
		}
		MyIpfp_checksum += inchar ;
	}
}


void MyIpsio_fp_checksum( unsigned char inchar )
{
	char hexVal = MyIphex_char_val(inchar) ;
	if (hexVal == -1)
	{
		MyIpsio_parse = &MyIpsio_newMsg ;
	}
	else if (MyIpfp_high_byte == -1)
	{
		MyIpfp_high_byte = hexVal * 16 ;
	}
	else
	{
		unsigned char v = MyIpfp_high_byte + hexVal ;
		if (v == MyIpfp_checksum)
		{
			flightplan_live_commit() ;
		}
		MyIpsio_parse = &MyIpsio_newMsg ;
	}
}



#endif // (NETWORK_USE_LOGO == 1)
#endif // ((USE_WIFI_NETWORK_LINK == 1) || (USE_ETHERNET_NETWORK_LINK == 1))
#endif // _MYIPLOGO_C_

