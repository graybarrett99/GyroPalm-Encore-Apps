// Begin AutoGenerated Includes - DO NOT EDIT BELOW
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
LV_IMG_DECLARE(alert64);
LV_IMG_DECLARE(clear_day64);
LV_IMG_DECLARE(clear_night64);
LV_IMG_DECLARE(cloudy64);
LV_IMG_DECLARE(fog64);
LV_IMG_DECLARE(partly_cloudy_day64);
LV_IMG_DECLARE(partly_cloudy_night64);
LV_IMG_DECLARE(rain64);
LV_IMG_DECLARE(sleet64);
LV_IMG_DECLARE(snow64);
LV_IMG_DECLARE(wind64);
LV_FONT_DECLARE(barlow_24);

lv_task_t* tmr_41335;
void lv_task_41335(struct _lv_task_t *data) {
    loop_41335();
}
// End AutoGenerated Includes - DO NOT EDIT ABOVE

typedef struct {
  lv_obj_t * lblCity;
  lv_obj_t * lblSummary;
  lv_obj_t * lblTempAct;
  lv_obj_t * lblTempFeel;
  lv_obj_t * lblPrecip;
  lv_obj_t * lblHumid;
  lv_obj_t * lblWind;
} str_WeatherMain_t;

static str_WeatherMain_t weatherObj;

float tempAct = 72.50;
float tempFeel = 70.10;
float windSpeed = 6.55;
float precip = 5.12;
float humid = 10.12;
char todaySummary[25] = "Partly Cloudy";
char weatherCity[25] = "Today's Weather";
char weatherIcon[25] = "partly-cloudy-day";

char hourlyName[8][25];
char weeklyName[8][25];
char hourlyIcon[8][25];
char weeklyIcon[8][25];
float hourlyTemp[8];
float weeklyTemp[8][2];
float hourlyPrecip[8];
float weeklyPrecip[8];


// Begin AutoGenerated Globals - DO NOT EDIT BELOW
enum Screen_41335 { SCR_41335_HOME, SCR_41335_HOURLY, SCR_41335_DAILY };	//Screen indexes
lv_obj_t *screen_41335[3];    //screen pointers
GyroPalmLVGL form_41335[3];   //screen helper methods
Screen_41335 curScreen_41335 = SCR_41335_HOME;    //default screen
// End AutoGenerated Globals - DO NOT EDIT ABOVE

GyroPalmCloud * gpcloud;

// Begin AutoGenerated Callbacks - DO NOT EDIT BELOW


static void btn_event_handler_41335(lv_obj_t * obj, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED) {
        String btnName = lv_list_get_btn_text(obj);
		Serial.printf("Clicked: %s\n", btnName);

        switch (curScreen_41335)
        {
            case SCR_41335_HOME:
                if (btnName == "Hourly") {
                    showApp_41335(SCR_41335_HOURLY);
                }
                if (btnName == "Weekly") {
                    showApp_41335(SCR_41335_DAILY);
                }
            break;

            case SCR_41335_HOURLY:
                if (btnName == "Today") {
                    showApp_41335(SCR_41335_HOME);
                }
                if (btnName == "Weekly") {
                    showApp_41335(SCR_41335_DAILY);
                }
            break;

            case SCR_41335_DAILY:
                if (btnName == "Today") {
                    showApp_41335(SCR_41335_HOME);
                }
                if (btnName == "Hourly") {
                    showApp_41335(SCR_41335_HOURLY);
                }
            break;

            default: break;
		}
	}
}

// End AutoGenerated Callbacks - DO NOT EDIT ABOVE

// Begin AutoGenerated Screens - DO NOT EDIT BELOW
void showApp_41335(int page) {
    if ((Screen_41335) page != curScreen_41335) {
        form_41335[curScreen_41335].removeBar();    //remove old StatusBar before proceeding
    }

    switch (page)
    {
		case SCR_41335_HOME:
        {
            //Draw screen UI
            curScreen_41335 = (Screen_41335) page;
            form_41335[curScreen_41335].init(screen_41335[curScreen_41335]);  //now defining screen items
            form_41335[curScreen_41335].createBar(barTask, lv_update_task);
            form_41335[curScreen_41335].setTime(gplm.getTime());

			weatherObj.lblCity = form_41335[curScreen_41335].createLabel(0, -70, weatherCity);    //show element

            weatherObj.lblSummary = form_41335[curScreen_41335].createLabel(0, -52, todaySummary);    //show element

			weatherObj.lblTempAct = form_41335[curScreen_41335].createLabel(-55, -25, "00.0 °F");    //show element
            lv_label_set_text_fmt(weatherObj.lblTempAct, "%0.1f °F", tempAct);

            lv_obj_set_style_local_text_font(weatherObj.lblTempAct, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &barlow_24);
            weatherObj.lblTempFeel = form_41335[curScreen_41335].createLabel(-50, 0, "Feels 00 °F");    //show element
            weatherObj.lblWind = form_41335[curScreen_41335].createLabel(-50, 20, "Wind: 0.00");    //show element
            lv_label_set_text_fmt(weatherObj.lblTempFeel, "Feels %0.1f °F", tempFeel);
            lv_label_set_text_fmt(weatherObj.lblWind, "Wind: %0.1f", windSpeed);

            if (strcmp(weatherIcon, "clear-day") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, clear_day64);    //show the icon
            }
            else if (strcmp(weatherIcon, "clear-night") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, clear_night64);    //show the icon
            }
            else if (strcmp(weatherIcon, "rain") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, rain64);    //show the icon
            }
            else if (strcmp(weatherIcon, "snow") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, snow64);    //show the icon
            }
            else if (strcmp(weatherIcon, "sleet") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, sleet64);    //show the icon
            }
            else if (strcmp(weatherIcon, "wind") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, wind64);    //show the icon
            }
            else if (strcmp(weatherIcon, "fog") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, fog64);    //show the icon
            }
            else if (strcmp(weatherIcon, "cloudy") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, cloudy64);    //show the icon
            }
            else if (strcmp(weatherIcon, "partly-cloudy-day") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, partly_cloudy_day64);    //show the icon
            }
            
            else if (strcmp(weatherIcon, "partly-cloudy-night") == 0) {
                form_41335[curScreen_41335].createImg(50, -25, partly_cloudy_night64);    //show the icon
            }
            else {
                form_41335[curScreen_41335].createImg(50, -25, alert64);    //show the icon
            }

            weatherObj.lblPrecip = form_41335[curScreen_41335].createLabel(50, 0, "Precip: 00%");    //show element
            weatherObj.lblHumid = form_41335[curScreen_41335].createLabel(50, 20, "Humid: 00%");    //show element
            lv_label_set_text_fmt(weatherObj.lblPrecip, "Precip: %0.1f%%", precip);
            lv_label_set_text_fmt(weatherObj.lblHumid, "Humid: %0.1f%%", humid);

			form_41335[curScreen_41335].createButton(-58, 84, "Hourly", btn_event_handler_41335, true, 98);    //show element
			form_41335[curScreen_41335].createButton(58, 84, "Weekly", btn_event_handler_41335, true, 98);    //show element

            form_41335[curScreen_41335].showScreen(ANIM_NONE);   //show the screen w/ no animation
        }
        break;

		case SCR_41335_HOURLY:
        {
            //Draw screen UI
            curScreen_41335 = (Screen_41335) page;
            form_41335[curScreen_41335].init(screen_41335[curScreen_41335]);  //now defining screen items
            form_41335[curScreen_41335].createBar(barTask, lv_update_task);
            form_41335[curScreen_41335].setTime(gplm.getTime());

			form_41335[curScreen_41335].createLabel(0, -70, "Hourly Weather");    //show element

            form_41335[curScreen_41335].createPage(0, 0, 228, 110, LV_SCRLBAR_MODE_AUTO);

            int iconX = 0;
            int iconY = 20;  //static
            for (int i = 0; i < 5; i++) {
                switch (i)
                {
                    case 0:
                        iconX = -87;
                    break;
                    case 1:
                        iconX = -22;
                    break;
                    case 2:
                        iconX = 43;
                    break;
                    case 3:
                        iconX = 108;
                    break;
                    case 4:
                        iconX = 173;
                    break;
                    default: break;
                }

                lv_obj_t * hrName = form_41335[curScreen_41335].createLabel(iconX, -30, hourlyName[i]);    //show element
                lv_obj_align(hrName, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 87, 0);

                lv_obj_t * hrIcon;

                if (strcmp(hourlyIcon[i], "clear-day") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, clear_day64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "clear-night") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, clear_night64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "rain") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, rain64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "snow") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, snow64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "sleet") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, sleet64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "wind") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, wind64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "fog") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, fog64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "cloudy") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, cloudy64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "partly-cloudy-day") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, partly_cloudy_day64);    //show the icon
                }
                else if (strcmp(hourlyIcon[i], "partly-cloudy-night") == 0) {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, partly_cloudy_night64);    //show the icon
                }
                else {
                    hrIcon = form_41335[curScreen_41335].createImg(iconX, iconY, alert64);    //show the icon
                }
                lv_obj_align(hrIcon, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 90, iconY);

                lv_obj_t * lblHourlyTemp = form_41335[curScreen_41335].createLabel(iconX, 55, "00.0 °F");    //show element
                lv_label_set_text_fmt(lblHourlyTemp, "%0.1f °F", hourlyTemp[i]);
                lv_obj_align(lblHourlyTemp, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 90, 55);
                
                lv_obj_t * lblHourlyRain = form_41335[curScreen_41335].createLabel(iconX, 55, "00.0%");    //show element
                lv_label_set_text_fmt(lblHourlyRain, "%0.1f%%", hourlyPrecip[i]);
                lv_obj_align(lblHourlyRain, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 92, 75);
            }

            form_41335[curScreen_41335].endPage();

			form_41335[curScreen_41335].createButton(-58, 84, "Today", btn_event_handler_41335, true, 98);    //show element
			form_41335[curScreen_41335].createButton(58, 84, "Weekly", btn_event_handler_41335, true, 98);    //show element

            form_41335[curScreen_41335].showScreen(ANIM_NONE);   //show the screen w/ no animation
        }
        break;

		case SCR_41335_DAILY:
        {
            //Draw screen UI
            curScreen_41335 = (Screen_41335) page;
            form_41335[curScreen_41335].init(screen_41335[curScreen_41335]);  //now defining screen items
            form_41335[curScreen_41335].createBar(barTask, lv_update_task);
            form_41335[curScreen_41335].setTime(gplm.getTime());

			form_41335[curScreen_41335].createLabel(0, -70, "Weekly Weather");    //show element

            form_41335[curScreen_41335].createPage(0, 0, 228, 110, LV_SCRLBAR_MODE_AUTO);

            int iconX = 0;
            int iconY = 20;  //static
            for (int i = 0; i < 5; i++) {
                switch (i)
                {
                    case 0:
                        iconX = -87;
                    break;
                    case 1:
                        iconX = -22;
                    break;
                    case 2:
                        iconX = 43;
                    break;
                    case 3:
                        iconX = 108;
                    break;
                    case 4:
                        iconX = 173;
                    break;
                    default: break;
                }

                lv_obj_t * wkName = form_41335[curScreen_41335].createLabel(iconX, -30, weeklyName[i]);    //show element
                lv_obj_align(wkName, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 87, 0);

                lv_obj_t * wkIcon;

                if (strcmp(weeklyIcon[i], "clear-day") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, clear_day64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "clear-night") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, clear_night64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "rain") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, rain64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "snow") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, snow64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "sleet") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, sleet64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "wind") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, wind64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "fog") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, fog64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "cloudy") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, cloudy64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "partly-cloudy-day") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, partly_cloudy_day64);    //show the icon
                }
                else if (strcmp(weeklyIcon[i], "partly-cloudy-night") == 0) {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, partly_cloudy_night64);    //show the icon
                }
                else {
                    wkIcon = form_41335[curScreen_41335].createImg(iconX, iconY, alert64);    //show the icon
                }
                lv_obj_align(wkIcon, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 90, iconY);

                lv_obj_t * lblWeeklyTempHi = form_41335[curScreen_41335].createLabel(iconX, 55, "00.0 °F");    //show element
                lv_label_set_text_fmt(lblWeeklyTempHi, "%i °F", (int)round(weeklyTemp[i][0]));
                lv_obj_align(lblWeeklyTempHi, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 90, 55);

                lv_obj_t * lblWeeklyTempLo = form_41335[curScreen_41335].createLabel(iconX, 55, "00.0 °F");    //show element
                lv_label_set_text_fmt(lblWeeklyTempLo, "%i °F", (int)round(weeklyTemp[i][1]));
                lv_obj_align(lblWeeklyTempLo, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 90, 75);
                
                lv_obj_t * lblWeeklyRain = form_41335[curScreen_41335].createLabel(iconX, 55, "00.0%");    //show element
                lv_label_set_text_fmt(lblWeeklyRain, "%0.1f%%", weeklyPrecip[i]);
                lv_obj_align(lblWeeklyRain, NULL, LV_ALIGN_IN_TOP_LEFT, iconX + 92, 95);
            }

            form_41335[curScreen_41335].endPage();

			form_41335[curScreen_41335].createButton(-58, 84, "Today", btn_event_handler_41335, true, 98);    //show element
			form_41335[curScreen_41335].createButton(58, 84, "Hourly", btn_event_handler_41335, true, 98);    //show element

            form_41335[curScreen_41335].showScreen(ANIM_NONE);   //show the screen w/ no animation
        }
        break;


        default: break;
    }
    form_41335[curScreen_41335].setBarCallback(hide_41335);
}
// End AutoGenerated Screens - DO NOT EDIT ABOVE

void getWeatherInfo(String zipCode) {
    if (zipCode.length() != 5) {
        return;
    }
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.setUserAgent("GyroPalmEncore");
        
        Serial.print("[HTTP] begin...\n");
        http.begin("http://app.gyropalm.com/api/central/weather"); //HTTP
        http.addHeader("Content-Type", "application/x-www-form_41335-urlencoded");
        //http.addHeader("Wearable-Id", "abc123");

        // start connection and send HTTP header
        String packet = "zip=" + zipCode;
        int httpCode = http.POST(packet);

        // httpCode will be negative on error
        if(httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTP] POST... code: %d\n", httpCode);

            // file found at server
            if(httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
                //Serial.println(payload);

                const size_t capacity = JSON_ARRAY_SIZE(3) + 2*JSON_ARRAY_SIZE(8) + 8*JSON_OBJECT_SIZE(4) + 8*JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(10) + 1500;
                DynamicJsonBuffer jsonBuffer(capacity);
                JsonArray& root = jsonBuffer.parseArray(payload);

                JsonObject& todayArr = root[0];
                strlcpy(todaySummary, todayArr["summary"], sizeof(todaySummary));
                strlcpy(weatherIcon, todayArr["icon"], sizeof(weatherIcon));
                tempAct = todayArr["actualTemp"]; // "76.730"
                tempFeel = todayArr["feelTemp"]; // "77.980"
                precip = todayArr["precipProb"]; // "0.0000"
                precip *= 100;
                humid = todayArr["humidity"]; // "0.8300"
                humid *= 100;
                windSpeed = todayArr["windSpeed"]; // "5.960"
                strlcpy(weatherCity, todayArr["city"], sizeof(weatherCity));

                JsonArray& hourlyArr = root[1];
                for (int i = 0; i < 5; i++) {
                    JsonObject& hourArr = hourlyArr[i];
                    strlcpy(hourlyName[i], hourArr["hour"], sizeof(hourlyName[i]));
                    strlcpy(hourlyIcon[i], hourArr["icon"], sizeof(hourlyIcon[i]));
                    hourlyTemp[i] = hourArr["actualTemp"];
                    hourlyPrecip[i] = hourArr["precipProb"];
                    hourlyPrecip[i] *= 100;
                }

                JsonArray& weeklyArr = root[2];
                for (int i = 0; i < 5; i++) {
                    JsonObject& weekArr = weeklyArr[i];
                    strlcpy(weeklyName[i], weekArr["day"], sizeof(weeklyName[i]));
                    strlcpy(weeklyIcon[i], weekArr["icon"], sizeof(weeklyIcon[i]));
                    weeklyTemp[i][0] = weekArr["highTemp"];
                    weeklyTemp[i][1] = weekArr["lowTemp"];
                    weeklyPrecip[i] = weekArr["precipProb"];
                    weeklyPrecip[i] *= 100;
                }

            }
        } else {
            Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
    }
}

void show_41335() {
	
    // Begin AutoGenerated Setup - DO NOT EDIT BELOW
    form[curScreen].clearLauncher();
    tmr_41335 = lv_task_create(lv_task_41335, 100, LV_TASK_PRIO_LOWEST, NULL);
    // End AutoGenerated Setup - DO NOT EDIT ABOVE

    gpcloud->getInstance();
    gpcloud->connectWiFi();

    getWeatherInfo("47906");
    showApp_41335(curScreen_41335);
}

void loop_41335() {
	
    // Begin AutoGenerated Loop - DO NOT EDIT BELOW
    // End AutoGenerated Loop - DO NOT EDIT ABOVE
}

void hide_41335(int barIntent) {
    if (barIntent == BAR_SWIPE_LEFT) {
        lv_task_del(tmr_41335); //removes the task
        showApp(SCR_HOME);
    }
}
