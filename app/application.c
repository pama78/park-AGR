//************************************************
//Name: agregator node
//Purpose: read from DAQ node the details, merge them and send to sigfox
//Author:  Pavel Majer
//
//Revisions: 
//
//************************************************

#include <application.h>
#include <radio.h>
#include <usb_talk.h>
#include <eeprom.h>
//#if CORE_MODULE
#include <sensors.h>
//#endif

#define APPLICATION_TASK_ID 0

static uint64_t my_id;
static bc_led_t led;
static bool led_state;
static bool radio_pairing_mode;

//pama nodes table
#define MAX_TABLE_LENGTH 10
struct
{
    int length;
    uint64_t id_nodes[10];

} table = { .length = 0 };

struct daq_type
{
    int    dist; //distance
    int    batt; //battery
    //tick of upd
};
struct daq_type daqs[10];
// nodes end



//pama nodes table end

//#if CORE_MODULE
//static struct
//{
//    bc_tick_t next_update;
//    bool mqtt;
//} lcd;

//static bc_module_relay_t relay_0_0;
//static bc_module_relay_t relay_0_1;

static void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param);
//static void lcd_button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param);
//#endif


int find_my_id_for_node_id(uint64_t node_id);
static void radio_event_handler(bc_radio_event_t event, void *event_param);
//static void led_state_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void led_state_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void relay_state_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void relay_state_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void module_relay_state_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void module_relay_pulse(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void module_relay_state_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void lcd_text_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void lcd_screen_clear(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void led_strip_color_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void led_strip_brightness_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void led_strip_compound_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void led_strip_effect_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
//static void led_strip_thermometer_set(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);

static void info_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void nodes_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void nodes_purge(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void nodes_add(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void nodes_remove(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);

static void scan_start(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void scan_stop(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void pairing_start(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void pairing_stop(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void automatic_pairing_start(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void automatic_pairing_stop(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);

static void alias_add(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void alias_remove(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);
static void alias_list(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub);

////pama nodes table
bool add_node_id_to_table(uint64_t node_id);
int find_my_id_for_node_id(uint64_t node_id);
bool load_table_from_eeprom(void);
bool save_table_to_eeprom(void);


const usb_talk_subscribe_t subscribes[] = {
    {"/info/get", info_get, 0, NULL},
    {"/nodes/get", nodes_get, 0, NULL},
    {"/nodes/add", nodes_add, 0, NULL},
    {"/nodes/remove", nodes_remove, 0, NULL},
    {"/nodes/purge", nodes_purge, 0, NULL},
    {"/scan/start", scan_start, 0, NULL},
    {"/scan/stop", scan_stop, 0, NULL},
    {"/pairing-mode/start", pairing_start, 0, NULL},
    {"/pairing-mode/stop", pairing_stop, 0, NULL},
    {"/automatic-pairing/start", automatic_pairing_start, 0, NULL},
    {"/automatic-pairing/stop", automatic_pairing_stop, 0, NULL},
    {"$eeprom/alias/add", alias_add, 0, NULL},
    {"$eeprom/alias/remove", alias_remove, 0, NULL},
    {"$eeprom/alias/list", alias_list, 0, NULL}
};

void application_init(void)
{
    bc_led_init(&led, GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    eeprom_init();

    usb_talk_init();
    bc_log_info("#application_init - usb_talk_init started\r\n");
    usb_talk_subscribes(subscribes, sizeof(subscribes) / sizeof(usb_talk_subscribe_t));

    bc_radio_init(BC_RADIO_MODE_GATEWAY);
    bc_radio_set_event_handler(radio_event_handler, NULL);

    //pama nodes table
    bc_log_info("#application_init - usb_talk_init started\r\n");

    //  if ( load_table_from_eeprom) 
    //  	{
     // 		    bc_log_info("#application_init - load_table_from_eeprom returned ok\r\n");
     // 	}

    bc_log_info("#application_init - CORE_MODULE V3\r\n");

    static bc_button_t button;
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    bc_led_pulse(&led, 2000);

    led_state = false;
    bc_log_info("#application_init - end\r\n");

}


void application_task(void)
{
    // bc_log_info ("#application_task initiated V3");
    bc_scheduler_plan_current_relative(500);

    /* pama nodes table
    load_table_from_eeprom;
    bc_log_error ("#load_table_from_eeprom - is ok");

     if (load_table_from_eeprom)
        {
         bc_log_error ("#load_table_from_eeprom - is ok");
         }
         else
         {
         bc_log_error ("#load_table_from_eeprom - is ok");
        }
       bool load_table_from_eeprom(void)
    */

}


void bc_radio_pub_on_buffer(uint64_t *id, uint8_t *buffer, size_t length)
{
    bc_log_info("#bc_radio_pub_on_buffer start\r\n");

    if (length < 1)
    {
        return;
    }

    //add to list, if it is not there yet
    //bc_log_info("!!!!bc_radio_pub_on_buffer - detected serial ( %i ) i \r\n",  (int)*id );
    bc_led_pulse(&led, 10);

    //decode nodes table (pozdeji se bude brat z eeprom, ted se priradi pri startu)
    int daq_id = find_my_id_for_node_id(*id);
    if (daq_id == -1)
    {
        //bc_log_info ("!! daq=-1, add node to table " );
        add_node_id_to_table(*id);
        //bc_log_info ("!! get new id just inserted to table " );
        daq_id = find_my_id_for_node_id(*id);
    }


    //show the payload
    float value;
    memcpy(&value, buffer + 1, sizeof(value));
    bc_log_info("---------------------------------------------------------------------");
    //bc_log_error("sent buffer is: serial:%i dist:%i lux:%i batVolt:%i batPerc:%i \n\r", (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0],    buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10]);
    bc_log_info("recieved buffer from DAQ %i serial:%llu dist:%i lux:%i batVolt:%i batPerc:%i \n\r", daq_id, *id, buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10]);


    daqs[daq_id].dist = buffer[4];
    daqs[daq_id].batt = buffer[6];

    //printout daqs
    for (int i = 0; i < table.length; i++)
    {
        bc_log_info("grabbed from table daq id %i distance: %i battery: %i ", i, daqs[i].dist, daqs[i].batt);
    }

    //možnost navázání logiky podle prvního bajtu - 
    /* if (length == 13)
     {
         switch (buffer[0])
         {
             // TODO: move to bc_radio_pub
             case RADIO_ACCELEROMETER_ACCELERATION:
             {
                 float x_axis, y_axis, z_axis;
                 memcpy(&x_axis, buffer + 1, sizeof(x_axis));
                 memcpy(&y_axis, buffer + 1 + sizeof(x_axis), sizeof(y_axis));
                 memcpy(&z_axis, buffer + 1 + sizeof(x_axis) + sizeof(y_axis), sizeof(z_axis));
                 usb_talk_publish_accelerometer_acceleration(id, &x_axis, &y_axis, &z_axis);
                 break;
             }
             default:
             {
                 break;
             }

         }
     }
     */
}


bool load_table_from_eeprom(void)
{
    return bc_eeprom_read(0, &table, sizeof(table));
}

bool save_table_to_eeprom(void)
{
    return bc_eeprom_write(0, &table, sizeof(table));
}

bool add_node_id_to_table(uint64_t node_id)
{
    //bc_log_info ("add_node_id_to_table -entered with node_id %llu ",  node_id );

    if (table.length == MAX_TABLE_LENGTH)
    {
        return false;
    }

    table.id_nodes[table.length] = node_id;

    table.length++;

    return true;
}

int find_my_id_for_node_id(uint64_t node_id)
{
    //  bc_log_info ("find_my_id_for_node_id -entered with node_id %llu ",  node_id );
    for (int i = 0; i < table.length; i++)
    {
        if (table.id_nodes[i] == node_id)
        {
            // bc_log_info ("find_my_id_for_node_id -about to return i: %i ", i );
            return i;
        }
    }

    bc_log_info("find_my_id_for_node_id -about to return -1 ");
    return -1;
}



//-----------------------------------------------------------------------------------------------------------------------------------
// BELOW IS INHERITED FUNCTIONS FROM bcf-gateway - for pairing
//-----------------------------------------------------------------------------------------------------------------------------------


static void radio_event_handler(bc_radio_event_t event, void *event_param)
{
    (void)event_param;
    bc_log_info("#radio_event_handler start\r\n");


    uint64_t id = bc_radio_get_event_id();

    if (event == BC_RADIO_EVENT_ATTACH)
    {
        bc_log_info("radio_event_handler BC_RADIO_EVENT_ATTACH\r\n");
        bc_led_pulse(&led, 1000);
        usb_talk_send_format("[\"/attach\", \"" USB_TALK_DEVICE_ADDRESS "\"]\n", id);
    }
    else if (event == BC_RADIO_EVENT_ATTACH_FAILURE)
    {
        bc_log_info("radio_event_handler BC_RADIO_EVENT_ATTACH_FAILURE\r\n");
        bc_led_pulse(&led, 5000);
        usb_talk_send_format("[\"/attach-failure\", \"" USB_TALK_DEVICE_ADDRESS "\"]\n", id);
    }
    else if (event == BC_RADIO_EVENT_DETACH)
    {
        bc_log_info("radio_event_handler BC_RADIO_EVENT_DETACH\r\n");
        bc_led_pulse(&led, 1000);
        usb_talk_send_format("[\"/detach\", \"" USB_TALK_DEVICE_ADDRESS "\"]\n", id);
    }
    else if (event == BC_RADIO_EVENT_INIT_DONE)
    {
        my_id = bc_radio_get_my_id();
        bc_log_info("BC_RADIO_EVENT_INIT_DONE. \r\n");

    }
    else if (event == BC_RADIO_EVENT_SCAN_FIND_DEVICE)
    {
        bc_log_info("radio_event_handler BC_RADIO_EVENT_SCAN_FIND_DEVICE\r\n");
        usb_talk_send_format("[\"/found\", \"" USB_TALK_DEVICE_ADDRESS "\"]\n", id);
    }

    bc_log_info("radio_event_handler end\r\n");

}

void bc_radio_on_info(uint64_t *id, char *firmware, char *version)
{
    bc_log_info("#bc_radio_on_info initiated");

    bc_led_pulse(&led, 10);

    usb_talk_send_format("[\"" USB_TALK_DEVICE_ADDRESS "/info\", {\"firmware\": \"%s\", \"version\": \"%s\"} ]\n", *id, firmware, version);
}


static void info_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#info_get initiated");

    (void)id;
    (void)sub;
    (void)payload;

    usb_talk_send_format("[\"/info\", {\"id\": \"" USB_TALK_DEVICE_ADDRESS "\", \"firmware\": \"" FIRMWARE "\", \"version\": \"" VERSION "\"}]\n", my_id);
}

static void nodes_get(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#nodes_get initiated");

    (void)id;
    (void)sub;
    (void)payload;

    uint64_t peer_devices_address[BC_RADIO_MAX_DEVICES];

    bc_radio_get_peer_id(peer_devices_address, BC_RADIO_MAX_DEVICES);

    usb_talk_publish_nodes(peer_devices_address, BC_RADIO_MAX_DEVICES);
    //pama
    bc_log_info("!!nodes get got %i", (int)peer_devices_address);
}


void _radio_node(usb_talk_payload_t *payload, bool(*call)(uint64_t))
{
    bc_log_info("#_radio_node initiated");

    char tmp[13];
    size_t length = sizeof(tmp);

    if (!usb_talk_payload_get_string(payload, tmp, &length))
    {
        return;
    }

    if (length == 12)
    {
        uint64_t id = 0;

        if (sscanf(tmp, "%012llx/", &id))
        {
            call(id);
        }
    }
}

static void nodes_add(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#nodes_add initiated");

    (void)id;
    (void)sub;
    _radio_node(payload, bc_radio_peer_device_add);
}

static void nodes_remove(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#nodes_remove initiated");

    (void)id;
    (void)sub;
    _radio_node(payload, bc_radio_peer_device_remove);
}

static void nodes_purge(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#nodes_purge initiated");

    (void)id;
    (void)payload;

    bc_radio_peer_device_purge_all();

    nodes_get(id, payload, sub);
}

static void scan_start(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#scan_start initiated");

    (void)id;
    (void)payload;
    (void)sub;

    bc_radio_scan_start();

    usb_talk_send_string("[\"/scan\", \"start\"]\n");
}

static void scan_stop(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#scan_stop initiated");

    (void)id;
    (void)payload;
    (void)sub;

    bc_radio_scan_stop();

    usb_talk_send_string("[\"/scan\", \"stop\"]\n");
}


static void pairing_start(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#pairing_start initiated");

    (void)id;
    (void)payload;
    (void)sub;

    radio_pairing_mode = true;

    bc_led_set_mode(&led, BC_LED_MODE_BLINK_FAST);

    bc_radio_pairing_mode_start();

    usb_talk_send_string("[\"/pairing-mode\", \"start\"]\n");
}

static void pairing_stop(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#pairing_stop initiated");

    (void)id;
    (void)payload;
    (void)sub;

    radio_pairing_mode = false;

    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    bc_radio_pairing_mode_stop();

    usb_talk_send_string("[\"/pairing-mode\", \"stop\"]\n");
}

static void automatic_pairing_start(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#automatic_pairing_start initiated");

    (void)id;
    (void)payload;
    (void)sub;

    bc_led_set_mode(&led, BC_LED_MODE_BLINK_FAST);

    bc_radio_automatic_pairing_start();

    usb_talk_send_string("[\"/automatic-pairing\", \"stop\"]\n");
}

static void automatic_pairing_stop(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#automatic_pairing_stop initiated");

    (void)id;
    (void)payload;
    (void)sub;

    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    bc_radio_automatic_pairing_stop();

    usb_talk_send_string("[\"/automatic-pairing\", \"stop\"]\n");
}

static void alias_add(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#alias_add initiated");

    (void)id;
    (void)sub;

    uint64_t node_id;

    if (!usb_talk_payload_get_key_node_id(payload, "id", &node_id))
    {
        return;
    }

    char name[EEPROM_ALIAS_NAME_LENGTH + 1];
    size_t length = sizeof(name);

    if (!usb_talk_payload_get_key_string(payload, "name", name, &length))
    {
        return;
    }

    eeprom_alias_add(&node_id, name);
}

static void alias_remove(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#alias_remove initiated");

    (void)id;
    (void)sub;

    uint64_t node_id;

    if (!usb_talk_payload_get_node_id(payload, &node_id))
    {
        return;
    }

    eeprom_alias_remove(&node_id);
}

static void alias_list(uint64_t *id, usb_talk_payload_t *payload, usb_talk_subscribe_t *sub)
{
    bc_log_info("#alias_list initiated");

    (void)id;
    (void)sub;

    int page;

    if (!usb_talk_payload_get_int(payload, &page))
    {
        return;
    }

    eeprom_alias_list(page);
}




static void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    bc_log_info("#button_event_handler initiated");

    (void)self;
    (void)event_param;

    if (event == BC_BUTTON_EVENT_PRESS)
    {
        static uint16_t event_count = 0;
        bc_log_error("#button_event_handler - BC_BUTTON_EVENT_PRESS");


        usb_talk_publish_event_count(&my_id, "push-button/-", &event_count);

        event_count++;
        bc_led_pulse(&led, 100);
    }
    else if (event == BC_BUTTON_EVENT_HOLD)
    {

        bc_log_error("#button_event_handler - BC_BUTTON_EVENT_HOLD");
        if (radio_pairing_mode)
        {
            bc_log_error("#button_event_handler - if radio_pairing_mode=true");

            radio_pairing_mode = false;
            bc_radio_pairing_mode_stop();
            bc_led_set_mode(&led, BC_LED_MODE_OFF);
            usb_talk_send_string("[\"/pairing-mode\", \"stop\"]\n");
        }
        else {
            bc_log_error("#button_event_handler - if radio_pairing_mode=true - else");

            radio_pairing_mode = true;
            bc_radio_pairing_mode_start();
            bc_led_set_mode(&led, BC_LED_MODE_BLINK_FAST);
            usb_talk_send_string("[\"/pairing-mode\", \"start\"]\n");
        }
    }
}

