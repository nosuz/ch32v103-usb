use crate::usb::handler::{ MAX_LEN };

const VENDER_ID_H: u8 = 0x66;
const VENDER_ID_L: u8 = 0x66;

const PRODUCT_ID_H: u8 = 0x56;
const PRODUCT_ID_L: u8 = 0x78;

const DEV_DESC: [u8; 18] = [
    18, // size of descriptor
    0x01, // device descriptor (0x01)
    0x00, // USB 2.0 in BCD
    0x02,
    0, // class code. 0: defined in interface
    0, // subclass. 0: unused
    0, // protocol. 0: unused
    MAX_LEN as u8, // max packet size for endpoint 0
    VENDER_ID_L, // vender ID
    VENDER_ID_H,
    PRODUCT_ID_L, // product ID
    PRODUCT_ID_H,
    0, // device version in BCD
    1,
    1, // index for manufacture string
    2, // index for product string
    3, // index for serial number string
    1, // number of possible configs
];

// https://github.com/RoboMaster/DevelopmentBoard-Examples/blob/master/USB/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
// also https://gist.github.com/tai/acd59b125a007ad47767

const USB_DESC_TYPE_CONFIGURATION: u8 = 0x02; // bDescriptorType: Configuration
const USB_DESC_TYPE_INTERFACE: u8 = 0x04; // bDescriptorType: Interface
const CS_INTERFACE: u8 = 0x24;
const USB_DESC_TYPE_ENDPOINT: u8 = 0x05;

const EP_IN: u8 = 0x80;
const EP_OUT: u8 = 0x00;

const CDC_CMD_EP: u8 = 1;
const CDC_OUT_EP: u8 = 2;
const CDC_IN_EP: u8 = 3;

// __ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
const CONFIG_DESC: [u8; 67] = [
    //Configuration Descriptor
    0x09, // bLength: Configuration Descriptor size
    USB_DESC_TYPE_CONFIGURATION, // bDescriptorType: Configuration
    // wTotalLength:no of returned bytes
    // total length: conf(9) + iface(9) + cdc(5 + 5 + 4 + 5) + ep1(7) + iface(9) +  + ep2(7 + 7)
    67, // or length of CONFIG_DESC
    0x00,
    0x02, // bNumInterfaces: 2 interface
    0x01, // bConfigurationValue: Configuration value
    0x00, // iConfiguration: Index of string descriptor describing the configuration
    0xc0, // bmAttributes: self powered
    0x32, // MaxPower 0 mA

    //---------------------------------------------------------------------------

    //Interface Descriptor
    0x09, // bLength: Interface Descriptor size
    USB_DESC_TYPE_INTERFACE, // bDescriptorType: Interface
    // Interface descriptor type
    0x00, // bInterfaceNumber: Number of Interface
    0x00, // bAlternateSetting: Alternate setting
    0x01, // bNumEndpoints: One endpoints used
    0x02, // bInterfaceClass: Communication Interface Class
    0x02, // bInterfaceSubClass: Abstract Control Model
    0x01, // bInterfaceProtocol: Common AT commands
    0x00, // iInterface:

    //Header Functional Descriptor
    0x05, // bLength: Endpoint Descriptor size
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x00, // bDescriptorSubtype: Header Func Desc
    0x10, // bcdCDC: spec release number
    0x01,

    //Call Management Functional Descriptor
    0x05, // bFunctionLength
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x01, // bDescriptorSubtype: Call Management Func Desc
    0x00, // bmCapabilities: D0+D1
    0x01, // bDataInterface: 1

    //ACM Functional Descriptor
    0x04, // bFunctionLength
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: Abstract Control Management desc
    0x02, // bmCapabilities

    //Union Functional Descriptor
    0x05, // bFunctionLength
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptorSubtype: Union func desc
    0x00, // bMasterInterface: Communication class interface
    0x01, // bSlaveInterface0: Data Class Interface

    //Endpoint 1 Descriptor
    0x07, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_ENDPOINT, // bDescriptorType: Endpoint
    CDC_CMD_EP + EP_IN, // bEndpointAddress
    0x03, // bmAttributes: Interrupt
    MAX_LEN as u8, // wMaxPacketSize:
    0,
    0x10, // bInterval:
    //---------------------------------------------------------------------------

    //Data class interface descriptor
    0x09, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_INTERFACE, // bDescriptorType:
    0x01, // bInterfaceNumber: Number of Interface
    0x00, // bAlternateSetting: Alternate setting
    0x02, // bNumEndpoints: Two endpoints used
    0x0a, // bInterfaceClass: CDC
    0x00, // bInterfaceSubClass:
    0x00, // bInterfaceProtocol:
    0x00, // iInterface:

    //Endpoint OUT Descriptor
    0x07, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_ENDPOINT, // bDescriptorType: Endpoint
    CDC_OUT_EP + EP_OUT, // bEndpointAddress
    0x02, // bmAttributes: Bulk
    MAX_LEN as u8, // wMaxPacketSize:
    0,
    0x00, // bInterval: ignore for Bulk transfer

    //Endpoint IN Descriptor
    0x07, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_ENDPOINT, // bDescriptorType: Endpoint
    CDC_IN_EP + EP_IN, // bEndpointAddress
    0x02, // bmAttributes: Bulk
    MAX_LEN as u8, // wMaxPacketSize:
    0,
    0x00, // bInterval: ignore for Bulk transfer
];

const LANG_IDS: [u8; 4] = [
    4, // length
    0x03, // string descriptor (0x03)
    0x09, // 0x0409 English (United States)
    0x04,
    // 0x11, // 0x0411 Japanese
    // 0x04,
];

const STR_1: [u8; 12] = [
    12, // length
    0x03, // string descriptor (0x03)
    b'n',
    0,
    b'o',
    0,
    b's',
    0,
    b'u',
    0,
    b'z',
    0,
];

const STR_2: [u8; 18] = [
    18, // length
    0x03, // string descriptor (0x03)
    b'C',
    0,
    b'H',
    0,
    b'3',
    0,
    b'2',
    0,
    b'V',
    0,
    b'1',
    0,
    b'0',
    0,
    b'3',
    0,
];

const STR_3: [u8; 12] = [
    12, // length
    0x03, // string descriptor (0x03)
    b'1',
    0,
    b'.',
    0,
    b'2',
    0,
    b'.',
    0,
    b'3',
    0,
];

const STR_4: [u8; 22] = [
    22, // length
    0x03, // string descriptor (0x03)
    b'c',
    0,
    b'd',
    0,
    b'c',
    0,
    b' ',
    0,
    b's',
    0,
    b'a',
    0,
    b'm',
    0,
    b'p',
    0,
    b'l',
    0,
    b'e',
    0,
];

pub const USB_DESC: [&[u8]; 7] = [
    &DEV_DESC,
    &CONFIG_DESC,
    &LANG_IDS,
    &STR_1,
    &STR_2,
    &STR_3,
    &STR_4,
];

#[derive(Copy, Clone)]
pub enum DescIndex {
    Device = 0,
    Configuration = 1,
    LangId = 2,
    String1 = 3,
    String2 = 4,
    String3 = 5,
    String4 = 6,
    Stalled = 0xff,
}