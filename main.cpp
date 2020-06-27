#include <src/LoRa_RasPi.h>
#include <iostream>
byte hello[32] = "HELLO";

int main (int argc, char *argv[]) {
	if (argc < 2) {
        cout << "Usage: argv[0] sender|rec [message]" << endl;
        exit(1);
    }
    
	LoRa MyLora;
	MyLora.begin(433E6);
	if (!strcmp("sender", argv[1])) {
		MyLora.configTransmitter();
		cout << "Send packets at SF" << MyLora.getSF() << " on " << MyLora.getLoRaFreq()/1000000 << "Mhz" << endl;
        cout << "----------------" << endl;

        if (argc > 2)
            strncpy((char *)hello, argv[2], sizeof(hello));

        while(1) {
            MyLora.txlora(hello, strlen((char *)hello));
            delay(5000);
        }
    } else {

        // radio init
        MyLora.configReceiver();
        cout << "Listening at SF" << MyLora.getSF() << " on " << MyLora.getLoRaFreq()/1000000 << "Mhz" << endl;
        cout << "----------------" << endl;
        while(1) {
            byte* message = new byte[256];
            if (MyLora.receive(message) & MyLora.dio0State()){
                cout << message << endl;
                cout << "----------------" << endl;
			}
            delete[] message;
            delay(1);
        }

    }
	
	
    return (0);
}
