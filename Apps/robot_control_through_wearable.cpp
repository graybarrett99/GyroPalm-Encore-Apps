/*
 *    Robot Control through Gestures
 *    Author: Grayson Barrett
 *    Co Authors: Corbin Newhard, Austin Ford, Alex Chui, William LaPlant,
 *    Richard Lopez, Jaylen Thomas, and Flevin Young
 *
 *    Created on: January 5, 2022
 *
 *    Last Modified on: March 29, 2022
 *
 *    Description: This program is the firmware for Purdue Capstone Team 26's 
 *    GyroPalm wearable. The wearable is activated with a snap and two shakes of
 *    the device. It then reads a flick in one of four directions. It passes the
 *    type of flick through realtime web sockets to a Trossen Robotics PincherX 100
 *    robotic arm to give movement commands to the arm.
 *
 */ 

#include <GyroPalmEngine.h>
#include <GyroPalmLVGL.h>

// Global variables needed for wearable firmware
GyroPalm *device;
GyroPalmEngine gplm("gp123456");    //declares a GyroPalm Engine object with wearableID
GyroPalmRealtime realtime;          //declares GyroPalm Realtime object

// Battery icon and update declarations
AXP20X_Class *power;
lv_task_t *barTask;
void lv_update_task(struct _lv_task_t *);

// Wearable screen definitions
enum Screen { SCR_HOME };	    //Screen indexes
lv_obj_t *screen[1];            //screen pointers
GyroPalmLVGL form[1];           //screen helper methods
Screen curScreen = SCR_HOME;    //default screen

// Programmer defined variables
bool isActive;
bool snapped;
int numShakes = 0;
long lastActivated = 0;
long lastSnapped = 0;

/*
 *  Update Battery Icon
 *
 *  This function updates the battery icon on the top bar of the wearable screen.
 *
 *  Imported from GyroPalm documentation
 */
void lv_update_task(struct _lv_task_t *data) {  
    int battPercent = power->getBattPercentage();
    bool isCharging = power->isChargeing();
    form[curScreen].updateBar(battPercent, isCharging);
    form[curScreen].setTime(gplm.getTime());
}

/* 
 *  Button Event Handler
 *
 *  This function handles button press events on the screen of the wearable.
 *
 *  Imported from GyroPalm documentation.
 */
static void btn_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED) {
        String btnName = lv_list_get_btn_text(obj);
		Serial.printf("Clicked: %s\n", btnName);

        switch (curScreen)
        {
            case SCR_HOME:
                if (btnName == "Connect") {
                    realtime.connect(gplm.myLicense);   //securely connect to GyroPalm Realtime
                }
                if (btnName == "Disconnect") {
                    realtime.disconnect();
                }
                if (btnName == "Home") {
                    realtime.sendSerial("Home");    // Send command to robot -- move to home position
                }
                if (btnName == "Sleep") {
                    realtime.sendSerial("Sleep");   // Send command to robot -- move to sleep position
                }
                if (btnName == "KeepAlive") {
                    realtime.sendSerial("KeepAlive");   // Keep connection to robot alive
                }
            break;

            default: break;
		}
	}
}

/*
 *  Screen Setup function
 *
 *  This function creates the screen UI for the wearable including the five buttons used for the project
 *
 *  Imported from GyroPalm documentation
 */
void showApp(int page) {
    if ((Screen) page != curScreen) {
        form[curScreen].removeBar();    //remove old StatusBar before proceeding
    }

    switch (page)
    {
		case SCR_HOME:
        {
            //Draw screen UI
            curScreen = (Screen) page;
            form[curScreen].init(screen[curScreen]);            //now defining screen items
            form[curScreen].createBar(barTask, lv_update_task); // Create battery bar
            form[curScreen].setTime(gplm.getTime());            // Set watch time
			form[curScreen].createLabel(0, -54, "Purdue Capstone Team 26");                     //create title
			form[curScreen].createButton(-58, 15, "Connect", btn_event_handler, true, 98);      //show element
			form[curScreen].createButton(58, 15, "Disconnect", btn_event_handler, true, 98);    //show element
			form[curScreen].createButton(-77, 84, "Home", btn_event_handler, true, 65);         //show element
			form[curScreen].createButton(0, 84, "Sleep", btn_event_handler, true, 65);          //show element
			form[curScreen].createButton(77, 84, "KeepAlive", btn_event_handler, true, 65);     //show element

            form[curScreen].showScreen(ANIM_NONE);   //show the screen w/ no animation
        }
        break;

        default: break;
    }
}

/*
 *  Gesture Control functions
 *
 *  These next three functions use the gesture recognition in the wearable to pass movement commands to the
 *  PincherX 100 robotic arm.
 *
 *  Activation is done with one snap, then two shakes of the wearable. After the wearable is activated, the user
 *  will flick their wrist in one direction. This flick is passed to the robot through the realtime web socket
 *  service, and is treated as a command in the robot code. The robot will then complete the instruction it is given
 *  autonomously. 
 *
 *  Imported from GyroPalm documentation, edited by author with help from co-authors
 */

void onRawSnap()
{
    snapped = true;         // Set snapped flag
    lastSnapped = millis(); // Set lastSnapped time to current time (for deactivation)
}

void onShake(int numShakes)
{  
    if((numShakes >= 3 && numShakes < 18) && snapped == true) {     // Snap, then shake the wearable 2-3 times to activate
        isActive = true;                        // Set active flag
        lastActivated = millis();               // Set lastActive time to current time (for deactivation)
        snapped = false;                        // Reset snapped flag
        form[curScreen].showIcon(BAR_GLANCE);   // Show an eye icon next to the battery to show wearable readiness
        realtime.sendSerial("active");          // Send serial command to robot (mostly for user)
    } else if(numShakes > 18){
        realtime.sendSerial("wave"); // If the user snaps, then waves the wearable a few times, the robot will wave back
    }
}

void onFlick(int direction)
{
    if(isActive){           // Flicks will only be allowed through if the watch has first been activated
        switch(direction)   // User then flicks in a direction -- up, down, left, or right
        {
            case FLICK_LEFT:
                realtime.sendSerial("lflick");  // Send left flick movement command to robot
            break;

            case FLICK_RIGHT:
                realtime.sendSerial("rflick");  // Send right flick movement command to robot
            break;

            case FLICK_UP:
                realtime.sendSerial("uflick");  // Send up flick movement command to robot
            break;

            case FLICK_DOWN:
                realtime.sendSerial("dflick");  // Send down flick movement command to robot
            break;
        }
    } else {    // do nothing if watch is not active
        return;
    }
}


/* 
 *  Realtime Connection functions
 *
 *  These two functions allow the operation of the realtime Web Socket system
 *  through establishing a connection and printing incoming strings to the serial terminal
 *
 *   For Purdue students: We had to use a wireless hotspot on one of our phones for WiFi. Follow
 *   steps below for this.
 *
 *  1. Connect to GyroPalm Encore wireless network from phone
 *  2. Go to 192.168.4.1 in the phone's web browser
 *  3. Specify the WiFi network for the watch to connect to (your hotspot)
 *
 *  Imported from GyroPalm documentation
 */

void onRealtimeConnection(bool isConnected)
{
    Serial.print("Connection: ");

    if (isConnected) {
        Serial.println("Connected");
    } else {
        Serial.println("Disconnected");
    }
}

void onRealtimeIncoming(String msg)
{
    Serial.print("Incoming: ");
    Serial.println(msg);
}


/*
 *  Setup and Loop functions
 *
 *  These two functions are the boot setup and main loop function for the wearable.
 *
 *  Imported from GyroPalm documentation and edited by author with help from co-authors
 */

void setup() {
	gplm.begin();
	delay(100);
	gplm.listenEvents(false);           //starts listening for events

	device = gplm.wearable;             //gives control to the developer to run device methods
	device->lvgl_begin();               //Initiate LVGL core
	device->bl->adjust(120);            //Lower the brightness
	power = gplm.power;		            //gives control to the developer to access power methods
	power->setChargeControlCur(500);    //enable fast charging

    // Activate snap, shake, and flick gesture awareness and control
    // Watch will look for only snaps, shakes, and flicks
    gplm.setRawSnapCallback(onRawSnap);
    gplm.setShakeCallback(onShake);
    gplm.setFlickCallback(onFlick);
    
    // Show the screen
	showApp(curScreen);

    realtime.connectWiFi();            //Establish connection with saved WiFi, or opens captive portal
    realtime.verboseFlag = true;       //Set to true if you want to see Realtime debug

    // Attach Realtime callbacks
    realtime.setConnectionCallback(onRealtimeConnection);
    realtime.setIncomingCallback(onRealtimeIncoming);
}

void loop() {
	realtime.loop();    //need this for GyroPalm Realtime to operate
    
    if (isActive) { //validate activation time
        if (millis() - lastActivated > 3000) {      //been more than 3 seconds
            isActive = false;                       //deactivate
            form[curScreen].hideIcon(BAR_GLANCE);   // Hide eye icon on watch face
        }
    }

    if (snapped) {  //when snap happens (first part of activation gestures)
        if (millis() - lastSnapped > 3000) {    //been more than 3 seconds
            snapped = false;                    // Deactivate snapped
        }                                       // Note: Snapped is deactivated automatically when isActive is true
    }
    
	lv_task_handler();
	delay(50);
}
