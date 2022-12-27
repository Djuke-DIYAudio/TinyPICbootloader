/*

    Unix Tiny BootLoader Loader 

    Copyright (C) 2006  José Antonio Robles Ordóñez.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

    beth.null@gmail.com

    Copyright (C) 2006 José Antonio Robles Ordóñez
    This program comes with ABSOLUTELY NO WARRANTY; for details type `utbll -h'.
    This is free software, and you are welcome to redistribute it
    under certain conditions;
    
*/    

/* 

This is the best present after a few days of work:

      [Identification try #0] Time out.
      [Identification try #1] Time out.
      [Identification try #2] Time out.
      [Identification try #3] Time out.
      [Identification try #4] Time out.
      [Identification try #5] Ok
              
              [Found] 16F87/16F88
              
              [000h - 020h] OK
              [020h - 040h] OK
              [040h - 060h] OK
              [F80h - FA0h] OK
              
      [Loaded succesfully] test.hex

*/

/*

   Greetings, resources and unclassified stuff ;)
   
      TinyBootloader: http://www.etc.ugal.ro/cchiculita/software/picbootloader.htm
         
            Thanks to Claudiu Chiculita, for his bootloader, for giving the .asm freely, and hey, what this program is without his work?.
            
      Serial port programming: http://www.easysw.com/~mike/serial/
      
            Michael R. Sweet, good serial programming guide under posix operating systems.
            
      CCS (Custom Computer Services, Inc.: http://www.ccsinfo.com/
      
            For providing licenses of their compilers for student and open source projects.
            
      Personal greets:
      
            Captain Morgan                : What I can say, ummm : "ymokauehmaeerettabhdarer"
            Gandano of the oregano forest : Some people have a natural way of doing things, hehe, when an account in your alpha?
            
*/            

#include <stdio.h>

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <tclap/CmdLine.h>

using namespace TCLAP;
using namespace std;

//#define __DEBUG__

struct TPic
{
   string sID;
   int    iID;
   int    iFlash;
};


// Map that holds the data of the suported microcontrollers
map<int,TPic> tPics;


// For handling the .hex file, we use an structure that represents continuous ranges of memory, for late sending it

class TChunk
{

protected:
   
   int iAddr;
   vector<char> tData;
   
   static char acOriginalReset[ 8 ];

public:
    
    int  GetAddr( void ) { return iAddr; };
    void SetAddr( int VALUE ) { iAddr = VALUE; };
    int  GetSize( void ) { return tData.size(); } ;
    void Append( char* BUFFER, int LENGTH );
    void Clear( void) { tData.clear(); }
    bool SendChunk( int PORT, int TRIES, int SLEEP_TIME );
    bool SendSpecialChunk( int PORT, int TRIES, int SLEEP_TIME, int FLASH );
    void FakeReset( void );
    
};

int SetupSerialPort( string TTY, int SPEED );
int SendWithACK    ( int PORT, string STR );
int CheckPIC       ( int PORT, int TRIES, int SLEEP_TIME );
int SendHex        ( int PORT, int PIC, const char* FILENAME, int TRIES, int SLEEP_TIME );
int ProcessHexLine ( const char* IN, unsigned int& ADDR, char* OUT );
int SendLine       ( int PORT, char* BUFFER, int ADDR, int LENGTH, int TRIES, int SLEEP_TIME );

int LoadHEX        ( const char* FILENAME, vector<TChunk>& CHUNKS );

void FillSupportedModels( void );

int main( int argc, char* argv[] )
{

   int iSerialPort;
   int iPicID;
     
   try
   {
      // Parse the command line
      
      // Command Line object.
      CmdLine tCmdLine( "Unix Tiny BootLoader Loader", ' ', "0.1" );
      
      // The arguments
      
      // Serial device
      ValueArg<string> tSerialDevice( "d", "device", "Serial Device, where the uC is connected to.(ej: /dev/ttyS0)", true, "//dev//ttyS0", "device driver" );
      
      // Serial device speed
      // Restrict the speed to the standard ones
      vector<int> tSpeeds;

      tSpeeds.push_back( 1200 );
      tSpeeds.push_back( 2400 );
      tSpeeds.push_back( 4800 );
      tSpeeds.push_back( 9600 );
      tSpeeds.push_back( 19200 );
      tSpeeds.push_back( 38400 );
      tSpeeds.push_back( 57600 );
      tSpeeds.push_back( 115200 );
      
      ValuesConstraint<int> tAllowedSpeeds( tSpeeds );

      ValueArg<int> tSpeed( "b", "bauds", "Serial communications baud rate", true, 115200, &tAllowedSpeeds );
      
      // File Name to load
      
      ValueArg<string> tFileName( "f", "file", ".HEX file to load.", true, "filename.hex", "filename.hex" );
      
      SwitchArg tCheckPic( "c", "check", "Check the communications and retrieve the target PIC.", false );
      
      // Add the arguments
      
      tCmdLine.add( tSerialDevice );
      tCmdLine.add( tSpeed );
      tCmdLine.add( tFileName );
      tCmdLine.add( tCheckPic );
      
      // Parse the command line
      
      tCmdLine.parse( argc, argv );
      
      iSerialPort = SetupSerialPort( tSerialDevice.getValue(), tSpeed.getValue() );
      
      if ( iSerialPort == -1 )
      {
         cerr << "Serial Port could not be opened" << endl;
         return -1;
      }
      
      // Load the supported models 
      
      FillSupportedModels( );
            
      // Load and send the hex file

      if ( SendHex( iSerialPort, iPicID, tFileName.getValue().c_str(), 10, 300000 ) >= 0)
      {
         cout << endl << "[SendHex succesfully] " << tFileName.getValue() << endl;
      }
      else
      {
         cout << endl << "[SendHex Failed] " << tFileName.getValue() << endl;
      }
      
      // Close communications
      
      close( iSerialPort );
      
   }
   catch( ArgException &e )
   {
      cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
   }
   
   return -1;
}


int LoadHEX( const char* FILENAME, vector<TChunk>& CHUNKS )
{
   char Buffer[ 256 ];

   ifstream f; // File stream
   string   l; // Line read from the file
   
   TChunk   tChunk;
   
   unsigned int iLineAddr;
   int iLineLength;
   int I = -1;
   
   f.open( FILENAME );
   
   if ( f.is_open( ) )
   {
      tChunk.SetAddr( -1 ); // Force a new chunk the next time
      
      while ( !f.eof( ) )
      {
         getline( f, l );
         
//         cout << l << endl;

         if ( l[0] == ';' )
         {
            continue;
         }
         
         iLineLength = ProcessHexLine( l.c_str(), iLineAddr, Buffer );

         if ( iLineLength >= 0 )
         {
            // Check if it is continuous
            if (  (I >= 0 ) && ( CHUNKS[I].GetAddr( ) + CHUNKS[I].GetSize( ) == iLineAddr ) )
            {
               // Continous
                  
               CHUNKS[ I ].Append( Buffer, iLineLength );
            }
            else
            {
               // Non continous, a new one
               //cout << "LoadHEX: non-continuous address: 0x" << hex << iLineAddr << dec << endl;
               
               I++;
                           
               tChunk.SetAddr( iLineAddr );
               tChunk.Append( Buffer, iLineLength );
               
               CHUNKS.push_back( tChunk );
               tChunk.Clear();
            }
         }
         else
         {
            if ( iLineLength == -1 ) // Last record
            {
               // There is no more chunks
               
               return CHUNKS.size();
            }
            else
            {
               // Error
               cout << "ProcessHexLine returned: " << iLineLength << endl;
               return -1;
            }
         }
      }
      
      f.close( );
      return 0;
   } else {
      cout << "file " << FILENAME << " could not be opened" << endl;
      return -1;
   }
}


int SendHex( int PORT, int PIC, const char* FILENAME, int TRIES, int SLEEP_TIME )
{
   int I;
  
   int iFlash;
    
   TChunk tChunk;
   vector<TChunk> tChunks;   

   // First load the .hex file
   cout << "Loading hex file" << endl;
   if ( LoadHEX( FILENAME, tChunks ) == -1 )
   {
      cout << "Failed to load hex file" << endl;
      return -1;
   } else {
      cout << "Hex file OK" << endl;
      cout << tChunks.size() << " chunks detected" << endl;
       for( vector<TChunk>::const_iterator tIter = tChunks.begin(); tIter != tChunks.end(); tIter++ )
       {
          tChunk = *tIter;
          cout << "- chunk @ 0x" << setfill('0') << setw(4) << hex << tChunk.GetAddr( ) << " -> " << dec << tChunk.GetSize( ) << " bytes" << endl;
       }
   }

   // Check the communications with the bootloader and retrieve the target PIC.
   cout << endl << "Please (re)boot the microcontroller now!!" << endl;
   PIC = CheckPIC( PORT, 20, 300000 );
   
   if ( PIC == -1 )
   {
      cout << "[PIC Identification Error]" << endl;
      return -1;
   }
	 
	if ( tPics.find( PIC ) == tPics.end() )
	{
	   cout << "[Model Not Supported] " << PIC << endl; 
      return -1;
	}

   cout << endl << "[Found] " << tPics[ PIC ].sID << endl << endl;
   
   // Prepare the new reset vector
     
   iFlash = tPics[ PIC ].iFlash; // For calculating where the loader is and inject the 5 first instructions at the begining and patch the original jump at the loader code section

   // Check if the HEX size fits in the microcontroller
   int hex_size = 0;
   for( vector<TChunk>::const_iterator tIter = tChunks.begin(); tIter != tChunks.end(); tIter++ )
   {
      tChunk = *tIter;
      int chunk_size = tChunk.GetAddr( ) + tChunk.GetSize( );
      if ( chunk_size > iFlash ) continue;
      if (chunk_size > hex_size) hex_size = chunk_size;
   }
   
   cout << showbase // show the 0x prefix
        << internal // fill between the prefix and the number
        << setfill('0') // fill with 0s
        << setw(4);
   cout << "Max PIC address: " << hex << iFlash - 200 << dec << endl;
   cout << "Max HEX address: " << hex << hex_size << dec << endl;
   if (hex_size > iFlash - 256) {
      cout << "Hex file does not fit in microcontroller" << endl;
      return -1;
   }

   // Send the data
   cout << "Send hex file to microcontroller" << endl;
  
   for( vector<TChunk>::const_iterator tIter = tChunks.begin(); tIter != tChunks.end(); tIter++ )
   {
      tChunk = *tIter;
      
      // Do some checks
      
      if ( tChunk.GetAddr( ) < 8 )
      {
         // Intercept the boot reset vector and fake it
         cout << "Store and replace the original boot reset vector" << endl;
         tChunk.FakeReset( );
      }

      if ( tChunk.GetAddr( ) + tChunk.GetSize( ) > iFlash )
      {
         continue; // Not send illegal chunks   
      }

      if ( tChunk.GetAddr( ) + tChunk.GetSize( ) > iFlash - 200)
      {
         cout << "HEX file too large, bootloader overwriting prevented. Firmware will not work properly!" << endl;
         return -1;
      }

      if ( !tChunk.SendChunk( PORT, TRIES, SLEEP_TIME ))
      {
         return -1;
      }
    }

   // Now patch the boot loader
   cout << "Patch the bootloader to start the program" << endl;
   if (!tChunk.SendSpecialChunk( PORT, TRIES, SLEEP_TIME, iFlash )) return -1;
   else return 1;
   
}

// return the length
int ProcessHexLine ( const char* IN, unsigned int& ADDR, char* OUT )
{
   #define _DELIMITER_ 0x00
   #define _LENGTH_    0x01
   #define _ADDR_      0x03
   #define _TYPE_      0x08
   #define _DATA_      0x09
   
   int iLength;
   int iAddr;
   int iCRC;
   
   char Dummy[ 5 ];
   int  I,iOffset;
     
   if ( IN[ _DELIMITER_ ] != ':' )
   {
      cout << "ProcessHexLine: delimiter was " << IN[_DELIMITER_] << " instead of ':'" << endl;
      return -2;
   }
   
   if ( IN[ _TYPE_ ] == '1' )
   {
      // End of file
      return -1;
   }
   
   if ( IN[ _TYPE_ ] != '0' &&  IN[ _TYPE_ ] != '4' )
   {
      cout << "ProcessHexLine: record type " << IN[_TYPE_] << " not supported" << endl;
      // Error
      return -2;
   }
   
  
   Dummy[ 0 ] = IN[ _LENGTH_ ];
   Dummy[ 1 ] = IN[ _LENGTH_ + 1 ];
   Dummy[ 2 ] = 0x00;
   
   iLength = strtol( Dummy, (char **)NULL, 16 );
   
   Dummy[ 0 ] = IN[ _ADDR_ ];
   Dummy[ 1 ] = IN[ _ADDR_ + 1 ];
   Dummy[ 2 ] = IN[ _ADDR_ + 2 ];
   Dummy[ 3 ] = IN[ _ADDR_ + 3 ];
   Dummy[ 4 ] = 0x00;

   unsigned int addr_lo = strtol( Dummy, (char **)NULL, 16 );
   unsigned int addr_hi = ADDR & 0xFFFF0000;
   
   if (IN[ _TYPE_ ] == '4') {
       if (addr_lo != 0) {
           cout << "ProcessHexLine: extended address other than 0x0000 not supported"<< endl;
           return -2;
       }
       Dummy[ 0 ] = IN[ _DATA_ ];
       Dummy[ 1 ] = IN[ _DATA_ + 1 ];
       Dummy[ 2 ] = IN[ _DATA_ + 2 ];
       Dummy[ 3 ] = IN[ _DATA_ + 3 ];
       Dummy[ 4 ] = 0x00;
       addr_hi = strtol( Dummy, (char **)NULL, 16 ) << 16;
   }

   ADDR = addr_hi + addr_lo;

#ifdef __DEBUG__
   //cout << "ProcessHexLine: address: " << hex << ADDR << dec << endl;
   //cout << "ProcessHexLine: length: " << iLength << endl;
#endif

   if (IN[ _TYPE_ ] == '4') return 0; // Don't process this record type

   // Put the hexadecimal chars into the buffer
   
   iOffset = _DATA_;
   Dummy[ 2 ] = 0x00;
   
   for ( I = 0; I < iLength; I++ )
   {
      Dummy[ 0 ] = IN[ iOffset++ ]; // High nibble
      Dummy[ 1 ] = IN[ iOffset++ ]; // Low nibble
      
      OUT[ I  ] = strtol( Dummy, (char **)NULL, 16 );
   }
   
   // CRC todotodo
  
   return iLength;
}

int my_write( int FLD, char* BUFFER, int N )
{

#ifdef __DEBUG__
   for ( int I = 0; I < N; I++ )
      printf("%02X ", (unsigned char)BUFFER[I] );
   return N;
#else
   return write( FLD, BUFFER, N );
#endif

}

int SendLine( int PORT, char* BUFFER, int ADDR, int LENGTH, int TRIES, int SLEEP_TIME )
{ 
   // Send the data following the Tiny BootLoader Protocol
   //cout << endl << "SendLine() address: " << hex << ADDR << dec << " length: " << LENGTH << endl;
   
   // First the address
   
   int I;
   char CRC = 0;
   char Dummy;
   char Buf[64];
   
   // First if LENGTH is not 64 fill up up to 64 with FF
   // Bytes with value FF will not overwrite data!!
   for( I = 0; I < 64; I++ )
   {
      if (I < LENGTH) Buf[I] = BUFFER[I];
      else Buf[ I ] = 0xFF;
   }
   
   LENGTH = 64;
   
   //Dummy = (ADDR >> 16) & 0xFF;
   Dummy = 0;
   I = my_write( PORT, &Dummy, 1 );
   CRC += Dummy;
   
   if ( I != 1 )
   {
      cerr << "Error Sending ADDRU: " << endl;
      return -1;
   }
   
   Dummy = (ADDR >> 8) & 0xFF;
   I = my_write( PORT, &Dummy, 1 );
   CRC += Dummy;
   
   if ( I != 1 )
   {
      cerr << "Error Sending ADDRH: " << endl;
      return -1;
   }
   
   
   Dummy = (ADDR) & 0xFF;
   I =  my_write( PORT, &Dummy, 1 );
   CRC += Dummy;  
   
   if ( I != 1 )
   {
      cerr << "Error Sending ADDRL: " << endl;
      return -1;
   }
   
   // The number of bytes
   
   Dummy = LENGTH;
   I = my_write( PORT, &Dummy, 1 );
   CRC += Dummy;
   
   if ( I != 1 )
   {
      cerr << "Error Sending DATA LENGTH: " << endl;
      return -1;
   }
   
   // The data
   
   I = my_write( PORT, Buf, LENGTH );
   
   if ( I != LENGTH )
   {
      cerr << "Error Sending DATA: "  << endl;
      return -1;
   }
   
   for( I = 0; I < LENGTH; I++ )
   {
      CRC += Buf[ I ];
   }
   
   CRC = (~CRC) + 1;
   
   I = my_write( PORT, &CRC, 1 );

   if ( I != 1 )
   {
      cerr << "Error Sending CRC: " << endl;
      return -1;
   }
   
    cout << showbase // show the 0x prefix
         << internal // fill between the prefix and the number
         << setfill('0'); // fill with 0s

   // Wait for the ACK
#ifndef __DEBUG__   
   do
   {
      I = read( PORT, &Dummy, 1 );
      
      if ( ( I == 1 ) && ( Dummy == 'K' ) )
      {
         cout << '\r';
         cout.flush();
         cout << "[" << hex << setw(4) << ADDR << " - " << ADDR + LENGTH - 1 << "] OK";
         
         return LENGTH;
      }
      else if ( ( I == 1 ) && ( Dummy == 'N' ) )
      {
          cerr << "CRC not OK!" << endl;
          return -1;
      }
      else
      {
         if ( I == -1 )
         {
            cerr << "Error Receiving Confirmation: " << endl;
         }
         else
         {
            usleep( SLEEP_TIME );
            TRIES--;
         }
      }
   } while ( TRIES > 0 );
#else
   return LENGTH;
#endif   
   cout << endl;
   
   return -1;

}

int SendWithACK( int PORT, char* BUFFER, int LENGTH, int TRIES, int SLEEP_TIME )
{
   int I,J;
   char C;
   
   // First empty the read buffer
#ifndef __DEBUG__   
   while ( read( PORT, &C, 1 ) != -1 );
#endif   
   // Send the data
   
   J = write( PORT, BUFFER, LENGTH );
   
   if ( J != LENGTH )
   {
      cerr << "Error sending: " << endl;
      
      return -1;
   }
   else
   {
      // Read & Wait if neccesary for the ack
      
      for ( I = 0; I < TRIES; I++ )
      {
#ifndef __DEBUG      
         J = read( PORT, &C, 1 );
#else
         J = 1;
         C = 'K';
#endif                  
         
         if ( (J == 1) && ( C == 'K'  ) )
         {
            return LENGTH;
         }
         
         usleep( SLEEP_TIME );
      }
      
      return -1;
   }
   
   
}

#ifdef __DEBUG__
int CheckPIC( int PORT, int TRIES, int SLEEP_TIME )
{
   return 0x4F; // Return a 18F2620/18F4620
}

#else
int CheckPIC( int PORT, int TRIES, int SLEEP_TIME )
{
   int I,J, iID = -1;
   
   char Buffer[ 2 ];
   
   // This is the protocol start point, send an 0x1C, retrieve the pic id and a 'K' as confirmation, so Hi Ho lets go.
   
   // First send the 0x1C byte until we get
       
   for( I = 0; I < TRIES; I++ )
   {
      Buffer[0] = 0xC1;
      J = write( PORT, Buffer, 1 );
      
      if ( J != 1 )
      {
         cout << "Error sending 0xC1." << endl;
         break;
      }
      
      if (I>0) cout << '\r';
      cout.flush();
      cout << "[Identification try #" << I << "] ";

      // Sleep for a while
     
      usleep( SLEEP_TIME );
      
      J = read( PORT, Buffer, 2 );
           
      if ( ( J == 2 ) )
      {
         if ( Buffer[ 1 ] != 'K' )
         {
            cout << "Found but bad answer: " << Buffer[ 1 ] << "." << endl;
         }
         else
         {
         
            if ( tPics.find( Buffer[ 0 ] ) != tPics.end() )
            {
               cout << "Ok      " ;
               //cout << "Found " << tPics[ Buffer[ 0 ] ].sID << "." << endl;
               iID = Buffer[ 0 ];
            }
            else
            {
               //cout << "Found but model not supported." << endl;
            }
            
            break;
         }
      }
      else
      {
         cout << "Time out" ;
      }
   }
   cout << endl;
  
   return iID;
}
#endif


int  SetupSerialPort( string TTY, int SPEED )
{
   struct termios tOptions;
   speed_t  tSpeed;

   int  iSerialPort;
   char Buffer;
   

   iSerialPort = open( TTY.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );

   if ( iSerialPort == -1 )
   {
      cerr << "\nOpening serial port device" << endl;

      return iSerialPort;
   }

   fcntl( iSerialPort, F_SETFL, 0 );

   // Set speed and comunications options

   if (tcgetattr( iSerialPort, &tOptions ) != 0)
   {
      cerr << "Error " << errno << " from tcgetattr" << endl;
   }
   
   switch ( SPEED )
   {
      case 1200:
      {
         tSpeed = B1200;
         break;
      }
      case 2400:
      {
         tSpeed = B2400;
         break;
      }
      case 4800:
      {
         tSpeed = B4800;
         break;
      }
      case 9600:
      {
         tSpeed = B9600;
         break;
      }
      case 19200:
      {
         tSpeed = B19200;
         break;
      }
      case 38400:
      {
         tSpeed = B38400;
         break;
      }
      case 57600:
      {
         tSpeed = B57600;
         break;
      }
      case 115200:
      {
         tSpeed = B115200;
         break;
      }
      default:
      {
         tSpeed = B0;
      }
      
   }

   cfsetispeed( &tOptions, tSpeed );
   cfsetospeed( &tOptions, tSpeed );
   
   //Enable the receiver and set local mode
    
   tOptions.c_cflag |= ( CLOCAL | CREAD );

   // Set 8N1
   
   tOptions.c_cflag &= ~PARENB;
   tOptions.c_cflag &= ~CSTOPB;
   tOptions.c_cflag &= ~CSIZE;
   tOptions.c_cflag |=  CS8;

   // Disable hardware flow control

   tOptions.c_cflag &= ~CRTSCTS;

   // Disable Software Flow Control
   
   tOptions.c_lflag &= ~( ICANON | ECHO  | ECHOE | ISIG );
   tOptions.c_iflag &= ~( IXON   | IXOFF | IXANY        );

   // Enable Raw Output
   tOptions.c_oflag &= ~OPOST;   
   cfmakeraw(&tOptions);
   
//   tcsetattr( iSerialPort, TCSANOW, &tOptions );
   
   // Set the timeouts
   tOptions.c_cc[ VMIN ]  = 0;
   tOptions.c_cc[ VTIME ] = 1; // One tenth of second of timeout for reads
   
   // Flush port, then apply attributes
   tcflush( iSerialPort, TCIFLUSH );
   if (tcsetattr( iSerialPort, TCSANOW, &tOptions ) != 0)
   {
      cerr << "Error " << errno << " from tcsetattr" << endl;
   }

   // Empty the read buffer, putting nonblocking the serial port, and later blocking
   
   fcntl( iSerialPort, F_SETFL, FNDELAY);
   
   while ( read( iSerialPort, &Buffer, 1 ) > 0 );
     
   fcntl(iSerialPort, F_SETFL, 0);

   return iSerialPort;
}


void FillSupportedModels( void )
{

   TPic p;
   
   p.sID    = "16F876A/16F877A";
   p.iID    = 0x31;
   p.iFlash = 0x2000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "16F873A/16F874A";
   p.iID    = 0x32;
   p.iFlash = 0x1000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "16F87/16F88";
   p.iID    = 0x33;
   p.iFlash = 0x1000;
   
   tPics[ p.iID ] = p;
   
   
   p.sID    = "18F252/18F452/18F2520/18F4520";
   p.iID    = 0x41;
   p.iFlash = 0x8000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F242/18F442/18F2420/18F4420";
   p.iID    = 0x42;
   p.iFlash = 0x4000;
    
   tPics[ p.iID ] = p;
   
   p.sID    = "18F258/18F458";
   p.iID    = 0x43;
   p.iFlash = 0x8000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F248/18F448";
   p.iID    = 0x44;
   p.iFlash = 0x4000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F1320/18F2320";
   p.iID    = 0x45;
   p.iFlash = 0x2000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F1220/18F2220";
   p.iID    = 0x46;
   p.iFlash = 0x1000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F4320/";
   p.iID    = 0x47;
   p.iFlash = 0x2000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F4220";
   p.iID    = 0x48;
   p.iFlash = 0x1000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F6720/18F8720";
   p.iID    = 0x4A;
   p.iFlash = 0x20000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F6620/18F8620";
   p.iID    = 0x4B;
   p.iFlash = 0x10000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F6520/18F8520";
   p.iID    = 0x4C;
   p.iFlash = 0x8000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F8680";
   p.iID    = 0x4D;
   p.iFlash = 0x10000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F2525/18F4525";
   p.iID    = 0x4E;
   p.iFlash = 0xC000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F2620/18F4620";
   p.iID    = 0x4F;
   p.iFlash = 0x10000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F2550/18F4550";
   p.iID    = 0x55;
   p.iFlash = 0x8000;
   
   tPics[ p.iID ] = p;
   
   p.sID    = "18F2455/18F4455";
   p.iID    = 0x56;
   p.iFlash = 0x6000;
   
   tPics[ p.iID ] = p;

}


/****************** class TChunk **********************/
void TChunk::Append( char* BUFFER, int LENGTH )
{

   for( int I = 0; I < LENGTH; I++ )
   {
      tData.push_back( BUFFER[ I ] );
   }
   
}
    
bool TChunk::SendChunk( int PORT, int TRIES, int SLEEP_TIME )
{
   // Send each chunk in blocks of 64 bytes
   // Make sure the start address is also at a multiple of 64 bytes
   cout << "Send Chunk " << hex << GetAddr() << "-" << GetAddr() + GetSize() << dec << endl;

   int I;
   int iSize = tData.size();
   
   // Detect if start address is at 64 byte multiple
   int iOffset = GetAddr() % 64;
   int iAddrNew = GetAddr() - iOffset;
   if (iOffset != 0)
   {
      //cout << "Adjusting start address " << hex << GetAddr() << " to " << iAddrNew << dec << endl;
      char Buf[64];
      for ( I = 0; I < 64; I ++)
      {
         if (I < iOffset) Buf[ I ] = 0xFF;
         else Buf[ I ] = tData[ I - iOffset ];
      }
         
      if ( SendLine( PORT, Buf, iAddrNew, 64, TRIES, SLEEP_TIME ) == -1 )
      {
         return false;
      }
   }
   for( I = 0; I < iSize; I += 64 )
   {
      if ( I < iOffset ) continue;
      if ( iSize - I + iOffset >= 64 )
      {
         if ( SendLine( PORT, &tData[ I - iOffset ], iAddr + I - iOffset, 64, TRIES, SLEEP_TIME ) == -1 )
         {
            return false;
         }
      }
      else
      {
         if ( SendLine( PORT, &tData[ I - iOffset ], iAddr + I - iOffset, iSize - I + iOffset, TRIES, SLEEP_TIME ) == -1 )
         {
            return false;
         }
      }
   }
   cout << endl;
   return true;
}
    
void TChunk::FakeReset( void )
{
   int J;
   int I;
      
   vector<char> tDummy;
   
   if ( iAddr > 8 )
   {
      return;
   }
   
   // Back up the reset vector for later patching the bootloader
   
   for ( I = 0; I < iAddr; I++ )
   {
      acOriginalReset[ I ] = 0x00;
   }
   
   J = 0;
   for ( I = iAddr; I < 8; I++, J++ )
   {
      acOriginalReset[ I ] = tData[ J ];
   }
   
   // Now offset the data 
      
   //tDummy.assign( tData.begin(), tData.end() );
   
   for ( I = 0; I < iAddr; I++ )
   {
      tDummy.push_back( 0x00 );
   }
   
   for ( I = 0; I < tData.size(); I++ )
   {
      tDummy.push_back( tData[ I ] );
   }
   
   
   iAddr = 0x0000;      
    
   // Patched code to call the bootloader for PIC18F
   tDummy[ 1 ] = 0xEF;
   tDummy[ 0 ] = 0xA0; // 0xEFA0 --> goto 0xFF38, where the loader starts
   tDummy[ 3 ] = 0xF0; 
   tDummy[ 2 ] = 0x7F; // 0xF07F --> same as original code
   tDummy[ 5 ] = 0x00; 
   tDummy[ 4 ] = 0xFF; // 0xFF00 --> same as original code
   tDummy[ 7 ] = 0x00; 
   tDummy[ 6 ] = 0xFF; // 0xFF00 --> same as original code
  

   // The first four instructions must call the loader, later the loader will call the original reset vector
//   tDummy[ 1 ] = 0x30;
//   tDummy[ 0 ] = 0x0F; // 0x300F --> movlw 0x0F (for selecting the higher bank of memory
   
//   tDummy[ 3 ] = 0x00;
//   tDummy[ 2 ] = 0x8A; // 0x008A --> movwf PCLATH (upper bits of PC)
      
   // Now the goto with the format: 101kkkkkkkkkkk (k's are the 11 lower bits of the jump
//   tDummy[ 5 ] = 0x2F;
//   tDummy[ 4 ] = 0xA0; // 0x2FA0 --> goto 0x07A0, where the loader starts
      
//   tDummy[ 7 ] = 0x00; 
//   tDummy[ 6 ] = 0x00; // 0x0000 --> nop   
      
   // With the fake reset we only select the upper bank of memory and make a jump into the entry point of the loader
   // at the end we send the original reset to an address into the microcontroller where the loader jumpms after it finish the work.
      
   tData = tDummy;
}

char TChunk::acOriginalReset[ 8 ];

bool TChunk::SendSpecialChunk( int PORT, int TRIES, int SLEEP_TIME, int FLASH )
{
   tData.clear();
   iAddr = FLASH -  200;
     
   // Then place the reset vector in the next 8 bytes
   for ( int I = 0; I < 8; I++ )
   {
      tData.push_back( acOriginalReset[ I ] );
   }
   
   return SendChunk( PORT, TRIES, SLEEP_TIME );
}

