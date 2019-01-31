// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include <stdlib.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 64          //image height
#define  IMWD 64         //image width

#define numWorkers 4       //number of cores working on the image

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

on tile[0] : in port buttons = XS1_PORT_4E; //input port for buttons
on tile[0] : out port LEDS = XS1_PORT_4F; //output port for LEDS

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6


//Adjusts the index for boundary values
//Used to implement the wrap around feature of the array
int Adjust(int number) {

    if(number == -1) {
        return IMHT - 1;
    }else if(number == IMHT) {
        return 0;
    }else {
        return number;
    }

}


//prints report of current state of game
void pauseReport(int rounds, int liveCells, unsigned int time) {

    printf("Rounds processed: %d \n", rounds);
    printf("Number of live cells: %d \n", liveCells);
    printf("Processing time elapsed: %u \n", time);

}

//READ BUTTONS and send button pattern to distributor, taken from antdefender skeleton code
void buttonListener(in port b, chanend toDistributor) {
  int r;
  while (1) {
    b when pinseq(15)  :> r;    // check that no button is pressed
    b when pinsneq(15) :> r;    // check if some buttons are pressed
    if ((r==13) || (r==14))     // if either button is pressed
    toDistributor <: r;             // send button pattern to distributor
  }
}

//Returns the number of alive neighbours a certain cell has
int FindNeighbours(unsigned char cells[IMWD][IMHT/numWorkers + 2], int cellX, int cellY) {

    //number of alive neighbours
    int count = 0;

    int adjustedX = cellX;
    int adjustedY = cellY;

    //Goes through the 8 cells around the specified cell and checks how many are alive
    for(int x = cellX - 1; x <= cellX + 1; x++) {
        for(int y = cellY - 1; y <= cellY +1; y++) {
            if(x == cellX && y == cellY) {
            }else {
                //adjusts x and y to implement the wrap around feature
                adjustedX = Adjust(x);
                adjustedY = Adjust(y);
                if(cells[adjustedX][adjustedY] == 0xFF) {
                    count++; //adds a count for each alive neighbour
                }
            }
        }
    }

    return count; //returns number of alive neighbours
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{

  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

  //Open PGM files
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s\n.", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
      //printf( "-%4.1d ", line[ x ] ); //show image values
    }
    //printf( "\n" );
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  return;

}

//Worker function, processes part of the image
void Worker(chanend distributor, int workerNum) {

        //number of rows that worker will operate on
        int workerHeight = IMHT / numWorkers;

        //declaring the cells and the output cells
        unsigned char cells[IMWD][IMHT/numWorkers+2];
        unsigned char cellsOut[IMWD][IMHT/numWorkers];

        //all workers in infinite while loop
        while(1) {

        //row above, row below and rows inbetween saved to cells
        for(int y = 0; y < workerHeight + 2; y++) {
            for(int x = 0; x < IMWD; x++) {
                distributor :> cells[x][y]; //takes the cells passed from the distributor
            }
        }

        //stores the cells into the cellsOut array in the appropriate place, since cells out is a smaller array
        for(int y = 0; y < workerHeight; y++) {
            for(int x = 0; x <IMWD; x++) {
                cellsOut[x][y] = cells[x][y+1];
            }
        }

        //workers calculates new cells for next grid according to game of life rules
        for(int y = 0; y < workerHeight; y++) {
            for(int x = 0; x < IMWD; x++) {

                //finds the number of alive neighbour cells
                int aliveNeighbours = FindNeighbours(cells, x, y + 1);


                    if(cells[x][y + 1] == 0xFF) {
                        if(aliveNeighbours == 2 || aliveNeighbours == 3) {
                            //if cell is alive and has 2 or 3 live neighbours it stays the same
                        }else {
                            cellsOut[x][y] = 0x0; //otherwise it dies
                        }
                    }else {
                        if(aliveNeighbours == 3) {
                          cellsOut[x][y] = 0xFF; //if a cell is dead and has 3 alive neighbours it comes to life
                        }
                    }

            }
        }

        //saves new cells to cells grid and sends to distributor
        for(int y = 0; y < workerHeight; y++) {
            for(int x = 0; x < IMWD; x++) {
                cells[x][y + 1] = cellsOut[x][y];
                distributor <: cellsOut[x][y];
            }
        }

        }

}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void Distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend workers[numWorkers], chanend toButtons, out port leds)
{
  uchar val;
  int workerHeight = IMHT / numWorkers; //number of cells each worker has to operate on
  unsigned char cells[IMHT][IMWD];
  unsigned char cellsOut[IMHT][IMWD]; //2 copies of grid so that output isnt incorrectly calculated when processing
  int totalAliveCells = 0; //number of alive cells

  int buttonIn = 0; //button states
  int buttonOut = 0;

  int loopCondition = 0; //holds value to decide whether to exit loop, eg on SW2 button press
  unsigned char flashLED = 0; //used to flash led on and off when processing

  int pausePrinted = 0; //stores whether game is already paused

  int round = 0; //round number

  int tilt = 0; //tilt value

  timer time;
  unsigned int start;
  unsigned int end;
  unsigned int difference = 0; // timer values


  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf("Press SW1 to start\n");
  //printf( "Waiting for Board Tilt...\n" );



  while(buttonOut == 0) { //starts after start button is pressed, stops after output button is pressed

      select {

          case toButtons :> buttonIn :

              if(buttonIn == 14) { //if start button then starts image read
                  loopCondition = 1;
                  printf("Reading image \n");
                  leds <: 4;
                  for( int y = 0; y < IMHT; y++ ) {   //go through all coloumns
                    for( int x = 0; x < IMWD; x++ ) { //go through all rows

                          c_in :> val;                    //read the pixel value
                          cells[x][y] = val;              //inputs pixel value to grid
                          cellsOut[x][y] = val;

                    }
                  }
              }else if(buttonIn == 13) { //if output button then changes the loop condition and exits the while loop to start outputting the result image
                  loopCondition = 0;
                  buttonOut = 1;
              }
              break;
          default:
              break;
      }

      fromAcc :> tilt; //takes tilt value from accelerometer

      if(loopCondition == 1) { //if input button has been pressed, starts processing

          flashLED ^= 1; //LED turns on/off every other round, provides flashing effect
          leds <: flashLED;
          time :> start;

          printf("Processing...\n" );
          for( int y = 0; y < (workerHeight + 2); y++ ) {   //go through 6 rows
            for( int x = 0; x < IMWD; x++ ) { //go through all rows

              //sends relevant part of grid to each worker
              workers[0] <: cells[x][Adjust(y-1)];
              workers[1] <: cells[x][Adjust(y+workerHeight-1)];
              workers[2] <: cells[x][Adjust((y+workerHeight*2)-1)];
              workers[3] <: cells[x][Adjust((y+workerHeight*3)-1)];
            }
          }

          //retrieves and puts together resulting image from each of the workers
          for(int worker = 0; worker < numWorkers; worker++) {
              for(int y = 0; y < workerHeight; y++) {
                  for(int x = 0; x < IMWD; x++) {
                     int height = worker*workerHeight;
                     workers[worker] :> cellsOut[x][height + y];
                  }
              }
          }


          //calculates total alive cells in grid and updates the original cells grid
          for(int y = 0; y < IMHT; y++) {
              for(int x = 0; x < IMWD; x++) {
                  cells[x][y] = cellsOut[x][y];
                  if(cells[x][y] == 0xFF) {
                      totalAliveCells++;
                  }
              }
          }

          round++;
          printf( "\n %d processing round(s) completed...\n", round);

          fromAcc :> tilt;
          while(tilt == 1) { //pauses the game when tilted
              leds <: 8;

              if(pausePrinted == 0) {//needed to make sure report information isn't spammed
                  time :> end;

                  difference += (end - start) / 100000;
                  start = end;

                  time :> start;

                  printf("Game paused \n");
                  pausePrinted = 1;
                  pauseReport(round, totalAliveCells, difference);



              }

              fromAcc :> tilt;
          }

          pausePrinted = 0;

      }
      totalAliveCells = 0;


  }

      leds <: 2;

      //outputs cells to dataoutstream
      for(int y = 0; y < IMHT; y++) {
          for(int x = 0; x < IMWD; x++) {
              c_out <: cellsOut[x][y];
          }
      }



}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{

  int res;
  uchar line[ IMWD ];

  //Open PGM file
  printf( "DataOutStream: Start...\n" );
  res = _openoutpgm( outfname, IMWD, IMHT );
  if( res ) {
    printf( "DataOutStream: Error opening %s\n.", outfname );
    return;
  }

  //Compile each line of the image and write the image line-by-line
  for( int y = 0; y < IMHT; y++ ) {
    for( int x = 0; x < IMWD; x++ ) {
      c_in :> line[ x ];
    }
    _writeoutline( line, IMWD );
    printf( "DataOutStream: Line written...\n" );
  }

  //Close the PGM image
  _closeoutpgm();
  printf( "DataOutStream: Done...\n" );
  return;

}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist) {

  i2c_regop_res_t result;
  char status_data = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {

    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    // sends 1 or 0 to distributor depending on whether board is tilted,
    if (x > 20) {
        toDist <: 1;
    } else  {
        toDist <: 0;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

i2c_master_if i2c[1];               //interface to orientation

chan c_inIO, c_outIO, c_control, workers[numWorkers], buttonToDist;    //channels for input, output, control, workers, and buttons


par {
    on tile[0] : buttonListener(buttons, buttonToDist); //thread to handle button inputs
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[0] : orientation(i2c[0],c_control);        //client thread reading orientation data
    on tile[0] : DataInStream("64x64.pgm", c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream("testout.pgm", c_outIO);       //thread to write out a PGM image
    on tile[0] : Distributor(c_inIO, c_outIO, c_control, workers, buttonToDist, LEDS);//thread to coordinate work on image
    on tile[1] : Worker(workers[0], 0); //worker1
    on tile[1] : Worker(workers[1], 1); //worker2
    on tile[1] : Worker(workers[2], 2); //worker3
    on tile[1] : Worker(workers[3], 3); //worker4
}

  return 0;
}
