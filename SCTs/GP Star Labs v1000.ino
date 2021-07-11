// Begin AutoGenerated Includes - DO NOT EDIT BELOW
#include <GyroPalmEngine.h>
#include <GyroPalmLVGL.h>
LV_IMG_DECLARE(clock64);
LV_IMG_DECLARE(control64);
LV_IMG_DECLARE(ifttt64);
LV_IMG_DECLARE(perform64);
LV_IMG_DECLARE(weather64);
LV_IMG_DECLARE(calculator64);
LV_IMG_DECLARE(calendar64);
LV_IMG_DECLARE(finance64);
LV_IMG_DECLARE(nav64);
LV_IMG_DECLARE(settings64);
LV_IMG_DECLARE(shutter64);
LV_IMG_DECLARE(totp64);
LV_IMG_DECLARE(deck64);
LV_IMG_DECLARE(metronome64);
LV_IMG_DECLARE(midi64);
// End AutoGenerated Includes - DO NOT EDIT ABOVE

// Begin AutoGenerated Globals - DO NOT EDIT BELOW
GyroPalm *device;
GyroPalmEngine gplm("gp123456");    //declares a GyroPalm Engine object with wearableID

AXP20X_Class *power;
lv_task_t *barTask;
void lv_update_task(struct _lv_task_t *);

enum Screen { SCR_HOME };	//Screen indexes
lv_obj_t *screen[1];    //screen pointers
GyroPalmLVGL form[1];   //screen helper methods
Screen curScreen = SCR_HOME;    //default screen
// End AutoGenerated Globals - DO NOT EDIT ABOVE

lv_app_icon appIcons[15];

// Begin AutoGenerated Callbacks - DO NOT EDIT BELOW
void lv_update_task(struct _lv_task_t *data) {
    int battPercent = power->getBattPercentage();
    bool isCharging = power->isChargeing();
    form[curScreen].updateBar(battPercent, isCharging);
    form[curScreen].setTime(gplm.getTime());     //update Time View

    //Auto-sleep on inactivity
    if (lv_disp_get_inactive_time(NULL) > 15000) {
        toggleSleep();
    }
}

static void btn_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED) {
        String btnName = lv_list_get_btn_text(obj);
		Serial.printf("Clicked: %s\n", btnName);

        switch (curScreen)
        {
            case SCR_HOME:
			
            break;

            default: break;
		}
	}
}
// End AutoGenerated Callbacks - DO NOT EDIT ABOVE

void toggleSleep()
{

        // We are sleeping the device when power button pressed
        device->displaySleep();
        device->powerOff();
        device->closeBL();        // switch off backlight
        device->stopLvglTick();   // Pause LVGL handler

        gpio_wakeup_enable ((gpio_num_t)AXP202_INT, GPIO_INTR_LOW_LEVEL);   //wake from power button interrupt
        gpio_wakeup_enable((gpio_num_t)BMA423_INT1, GPIO_INTR_HIGH_LEVEL);  //wake from sensor interrupt
        esp_sleep_enable_gpio_wakeup();

        //Disable unnecessary motion feature interrupts
        device->bma->enableStepCountInterrupt(false);
        device->bma->enableWakeupInterrupt(false);    //isDoubleClick
        device->bma->enableActivityInterrupt(false);  //idle, walking, running

        esp_light_sleep_start();
        //wake time
        delay(10);

        power->setPowerOutPut(AXP202_EXTEN, true);
        power->setPowerOutPut(AXP202_LDO3, true);
        power->setPowerOutPut(AXP202_LDO2, AXP202_ON);

        device->displayWakeup();
        device->openBL();
        device->startLvglTick();      // LVGL handler
        lv_disp_trig_activity(NULL);    //trigger user activity

        //Re-enable previously disabled motion feature interrupts
        device->bma->enableStepCountInterrupt(true);
        device->bma->enableWakeupInterrupt(true);    //isDoubleClick
        device->bma->enableActivityInterrupt(true);  //idle, walking, running
}
void onPwrQuickPress()
{
    /*
    After the AXP202 interrupt is triggered, the interrupt status must be cleared,
    * otherwise the next interrupt will not be triggered
    */
    power->clearIRQ();
    device->bma->readInterrupt();

    toggleSleep();
}

void bar_handler(int barIntent)
{
    switch (barIntent)
    {
        case BAR_PRESSING:
            Serial.println("Pressing");
        break;

        case BAR_SWIPE_LEFT:
            Serial.println("Swipe left");
            showApp(SCR_HOME);
        break;

        case BAR_SWIPE_RIGHT:
            Serial.println("Swipe right");
        break;

        case BAR_RELEASED:
            Serial.println("Released");
        break;
    }
}

// Begin AutoGenerated Screens - DO NOT EDIT BELOW

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
            form[curScreen].init(screen[curScreen]);  //now defining screen items
            form[curScreen].createBar(barTask, lv_update_task);
            form[curScreen].setTime(gplm.getTime());

            int numApps = 0;

            appIcons[numApps].img_src = &clock64;
            appIcons[numApps].txt = "Clock";
            appIcons[numApps].userDefinedAppStart = show_123456;
            numApps++;

            appIcons[numApps].img_src = &control64;
            appIcons[numApps].txt = "Control";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &perform64;
            appIcons[numApps].txt = "Perform";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &ifttt64;
            appIcons[numApps].txt = "IFTTT";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &weather64;
            appIcons[numApps].txt = "Weather";
            appIcons[numApps].userDefinedAppStart = show_weather;
            numApps++;

            appIcons[numApps].img_src = &settings64;
            appIcons[numApps].txt = "Settings";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &calendar64;
            appIcons[numApps].txt = "Calendar";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &finance64;
            appIcons[numApps].txt = "Finance";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &nav64;
            appIcons[numApps].txt = "Nav";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &shutter64;
            appIcons[numApps].txt = "Shutter";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &calculator64;
            appIcons[numApps].txt = "Calculator";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &totp64;
            appIcons[numApps].txt = "TOTP";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &deck64;
            appIcons[numApps].txt = "Deck";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &metronome64;
            appIcons[numApps].txt = "Tempo";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            appIcons[numApps].img_src = &midi64;
            appIcons[numApps].txt = "MIDI";
            appIcons[numApps].userDefinedAppStart = startApp;
            numApps++;

            form[curScreen].createLauncher(appIcons, numApps, 6);

            form[curScreen].showScreen(ANIM_NONE);   //show the screen w/ no animation
            
        }
        break;
        
        default: break;
    }
    form[curScreen].setBarCallback(bar_handler);
}

// End AutoGenerated Screens - DO NOT EDIT ABOVE

void startApp()
{
    form[curScreen].clearLauncher();
    Serial.println("app start event");
}

void setup() {
	
    // Begin AutoGenerated Setup - DO NOT EDIT BELOW
	gplm.begin();
	delay(100);
	gplm.listenEvents(false);    //starts listening for events

    gplm.setPwrQuickPressCallback(onPwrQuickPress);
    
    delay(1000);
	device = gplm.wearable; //gives control to the developer to run device methods
	device->lvgl_begin();   //Initiate LVGL core
	device->bl->adjust(120);    //Lower the brightness
	power = gplm.power;		//gives control to the developer to access power methods
	power->setChargeControlCur(500);    //enable fast charging

	// End AutoGenerated Setup - DO NOT EDIT ABOVE

    showApp(curScreen);
}

void loop() {
	
    // Begin AutoGenerated Loop - DO NOT EDIT BELOW
	lv_task_handler();
	delay(50);
	// End AutoGenerated Loop - DO NOT EDIT ABOVE
}
