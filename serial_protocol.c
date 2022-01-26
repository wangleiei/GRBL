#include "serial_protocol.h"

#define LINE_BUFFER_SIZE 50

static uint8_t line[LINE_BUFFER_SIZE];
static uint8_t char_counter;

static void status_message(int32_t status_code);

static void status_message(int32_t status_code){
  switch(status_code){
    case GCSTATUS_OK:
        GrblMeth.printPgmString("ok\n\r"); break;
    case GCSTATUS_BAD_NUMBER_FORMAT:
        GrblMeth.printPgmString("error: Bad number format\n\r"); break;
    case GCSTATUS_EXPECTED_COMMAND_LETTER:
        GrblMeth.printPgmString("error: Expected command letter\n\r"); break;
    case GCSTATUS_UNSUPPORTED_STATEMENT:
        GrblMeth.printPgmString("error: Unsupported statement\n\r"); break;
    case GCSTATUS_FLOATING_POINT_ERROR:
        GrblMeth.printPgmString("error: Floating point error\n\r"); break;
    default:
        GrblMeth.printPgmString("error: ");
        printInteger(status_code);
        GrblMeth.printPgmString("\n\r");
    }
}
void sp_init() 
{
  // beginSerial(BAUD_RATE);  
  GrblMeth.printPgmString("\r\nGrbl " GRBL_VERSION);
  GrblMeth.printPgmString("\r\n");  
}

void sp_process()
{
  uint8_t c;
  while((c = GrblMeth.ReadChar()) != -1) 
  {
    if((char_counter > 0) && ((c == '\n') || (c == '\r'))) {  // Line is complete. Then execute!
      line[char_counter] = 0; // treminate string
      status_message(gc_execute_line(line));
      char_counter = 0; // reset line buffer index
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}
