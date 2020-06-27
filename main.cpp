#include <src/LoRa_RasPi.h>
byte hello[32] = "HELLO";


int main (int argc, char *argv[]) {
	if (argc < 2) {
        printf ("Usage: argv[0] sender|rec [message]\n");
        exit(1);
    }
    
	LoRa MyLora;
	MyLora.begin(433E6);
	if (!strcmp("sender", argv[1])) {
		MyLora.configTransmitter();
        printf("Send packets at SF%i on %.6lf Mhz.\n", MyLora.getSF(),(double)(MyLora.getLoRaFreq())/1000000);
        printf("------------------\n");

        if (argc > 2)
            strncpy((char *)hello, argv[2], sizeof(hello));

        while(1) {
            MyLora.txlora(hello, strlen((char *)hello));
            delay(5000);
        }
    } else {

        // radio init
        MyLora.configReceiver();
        printf("Listening at SF%i on %.6lf Mhz.\n", MyLora.getSF(),(double)(MyLora.getLoRaFreq())/1000000);
        printf("------------------\n");
        while(1) {
            //MyLora.receivepacket();
            byte* message = new byte[256];
            if (MyLora.receive(message) & MyLora.dio0State()){
                printf("Payload: %s\n", message);
	    }
            delete[] message;
            delay(1);
        }

    }
	
	
	return (0);
}
