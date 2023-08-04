// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "YOUR SSID";
const char* password = "YOUR WIFI PASSWORD";

// To send Emails using Gmail on port 465 (SSL), you need to create an app password: https://support.google.com/accounts/answer/185833
#define emailSenderAccount    "yoursender@email.here"
#define emailSenderPassword   "yoursendermailpassword"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "ESP32-CAM Photo Captured"
#define emailRecipient        "yourrecipient@email.here"