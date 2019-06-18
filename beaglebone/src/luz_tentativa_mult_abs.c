#include <rc/i2c.h>
#include <stdio.h>
#include <rc/time.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <math.h>


#define BH1750_BUS 	1
#define BH1750_HADD	0x5C
#define BH1750_LADD     0x23	
#define HRES_CONT	0x10 	//Constinuosly H-Resolution Mode
#define HRES2_CONT	0x11	//Constinuosly H-Resolution Mode2	
#define LRES_CONT	0x13	//Constinuosly L-Resolution Mode


#define OPCODE_WRITE    0X10

#define KP_LIGHT        2
#define EULER           2.718282


float mult_abs_val (float a, float b) 
{
        if (a == 0.0) return b;
        else if (b == 0.0) return a;
        else 
        {
                return (pow(EULER, fabs(log(a/b)))); 
        }
}

uint16_t dataH, dataL;
static int running = 0;
//uint8_t data;
// initialize the bus

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


int main()
{
        // make sure the bus is not currently in use by another thread
        // do not proceed to prevent interfering with that process
        if(rc_i2c_get_lock(BH1750_BUS)){
        	printf("WARNING: i2c bus claimed by another thread\n");
        	printf("Continuing anyway.\n");
        }
        
        // initialize the bus
        if(rc_i2c_init(BH1750_BUS, BH1750_HADD)<0){
        	printf("ERROR: failed to initialize i2c bus\n");
        	return -1;
        	
        }
       if(rc_i2c_init(BH1750_BUS, BH1750_LADD)<0){
        	printf("ERROR: failed to initialize i2c bus\n");
        	return -1;
        	
        }
        running = 1;
        // claiming the bus does no guarantee other code will not interfere
        // with this process, but best to claim it so other code can check
        // like we did above
        rc_i2c_lock_bus(BH1750_BUS);
        

        
                // write the measurement command
        //printf("%d\n", rc_i2c_write_word(BH1750_BUS, BH1750_HADD,w));
        
       // printf("\t%d\n", write(rc_i2c_get_fd(BH1750_BUS), writeData ,2));
        
       rc_i2c_init(BH1750_BUS, BH1750_HADD);
        rc_usleep(120000);
        for(int i=0; i<100000; i++){
            rc_i2c_set_device_address(BH1750_BUS, BH1750_HADD);
            rc_i2c_read_word(BH1750_BUS, BH1750_HADD, &dataH);
            //rc_usleep(120000);
            rc_i2c_set_device_address(BH1750_BUS, BH1750_LADD);
            rc_i2c_read_word(BH1750_BUS, BH1750_LADD, &dataL);
            
            //if(rc_i2c_read_word(BH1750_BUS, BH1750_LADD, &dataL)<0){
    	        //printf("ERROR: in rc_bmp_init can't read status byte from barometer\n");
    	//rc_i2c_unlock_bus(BH1750_BUS);
    	//return -1;
           // }
        	printf("Luminosity high %.2f, low %.2f\n", dataH/1.2, 1.4*dataL);
        	float dist = mult_abs_val(dataH, dataL);
        	printf("dist %.2f\n\n", dist);
        	float err_light = dist <= 1.5) ? 0.0 : ((dataL*1.4)-(dataH/1.2));
                float ref = err_light*KP_LIGHT;
                printf("ref %.2f\n\n", ref);
        	fflush(stdout);
        	rc_usleep(2000000);
        	
                
        }//ns the number of bytes that were written. If value is negative, then the system call returned an error.

        
       // printf("Data in hex: %X\n", data);
        //ns the number of bytes that were written. If value is negative, then the system call returned an error.

        // release control of the bus
        rc_i2c_unlock_bus(BH1750_BUS);
        
        // wait for bmp to finish it's internal initialization
        rc_usleep(50000);
       return 0;
}