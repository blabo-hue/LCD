/*
 * src/main.cpp — MixMate LCD Display
 *
 * ESP32 + ST7796 TFT (320×480 portrait) + ESP-NOW receiver
 *
 * Receives text commands from the main MixMate controller ESP32 via ESP-NOW:
 *   USERNAME:<name>       — update the welcome screen name
 *   START                 — clear old ingredients, enter DISPENSING state
 *   DISPENSE:<name>,<ml>  — add an ingredient and start its progress timer
 *   COMPLETE              — show completion screen then return to IDLE
 *
 * Hardware: ESP32 DevKit, ST7796 SPI TFT via TFT_eSPI (pin config in User_Setup.h)
 * Compile: PlatformIO with espressif32 + Arduino framework
 */

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <esp_now.h>
#include <WiFi.h>

// ── Build-time feature flags ──────────────────────────────────────────────────
// Comment out a line to disable that feature in the production build.
#define SERIAL_DEBUG   // Serial monitor command input for bench testing
#define SCREEN_TEST    // R/G/B flash on boot — confirms display hardware works

// ─────────────────────────────────────────────────────────────────────────────
// Screen constants  (portrait orientation: 320 wide × 480 tall)
// ─────────────────────────────────────────────────────────────────────────────
#define SCR_W       320
#define SCR_H       480
#define HDR_H        50   // header strip (gradient band at top)
#define MARGIN        8   // outer card margin and inner content margin

// Progress bar dimensions
#define BAR_H        14                         // bar height in pixels
#define BAR_W       (SCR_W - 4 * MARGIN)       // 288 px — content inset from card edge

// ─────────────────────────────────────────────────────────────────────────────
// Colour palette (RGB565)
//   Cyan   #00B4D8  →  R=0, G=45, B=27  → (0<<11)|(45<<5)|27  = 0x05BB
//   Fuchsia #FF00FF →  R=31,G=0, B=31   → (31<<11)|(0<<5)|31  = 0xF81F
//   Dark bg #080810 →  R=1, G=2, B=2    → 0x0842
//   Card bg #0D1018 →  R=1, G=4, B=3    → 0x0883
//   Bar trk #201828 →  R=4, G=3, B=5    → 0x2065
// ─────────────────────────────────────────────────────────────────────────────
#define C_GRAD_TOP   0x05BB   // Cyan  (brand colour 1)
#define C_GRAD_BOT   0xF81F   // Fuchsia (brand colour 2)
#define C_DARK_BG    0x0842   // Near-black background
#define C_CARD_BG    0x0883   // Dark navy card fill
#define C_BAR_TRACK  0x2065   // Muted purple bar track
#define C_WHITE      0xFFFF
#define C_GRAY       0x8410   // Mid-grey for secondary text
#define C_GREEN      0x07E0   // Pure green for "Done!"

// ─────────────────────────────────────────────────────────────────────────────
// Flow rate
//   72 mL/min  →  1 mL ≈ 833 ms  →  duration_ms = ml × 833
// ─────────────────────────────────────────────────────────────────────────────
#define MS_PER_ML    833UL

// Force-stop button: GPIO 0 (BOOT button on most ESP32 devkits, active-low)
#define STOP_BTN_PIN  0

// On-screen FORCE STOP touch button (bottom of display, visible during dispensing)
#define STOP_BTN_H   56                    // button height in pixels
#define STOP_BTN_Y   (SCR_H - STOP_BTN_H) // button top Y = 424

// Touch detection threshold: bottom fifth of the 480 px screen = y >= 384.
// We check BOTH the forward and inverted Y mapping so the hit region is correct
// regardless of whether this board's XPT2046 Y-axis runs top-to-bottom or bottom-to-top.
#define TOUCH_BOTTOM_FIFTH  (SCR_H * 4 / 5)   // 384 px

// Raw XPT2046 calibration — adjust if taps feel offset.
// Tap each corner and print p.x / p.y via Serial to find your exact values.
#define TS_MINX  300
#define TS_MAXX 3800
#define TS_MINY  300
#define TS_MAXY 3800

// ─────────────────────────────────────────────────────────────────────────────
// Limits
// ─────────────────────────────────────────────────────────────────────────────
#define MAX_DISP      6          // max concurrent ingredients
#define COMPLETE_HOLD 3000UL    // ms to show the "Complete!" screen

// ─────────────────────────────────────────────────────────────────────────────
// State machine
// ─────────────────────────────────────────────────────────────────────────────
enum DisplayState { ST_IDLE, ST_DISPENSING, ST_COMPLETE };

// ─────────────────────────────────────────────────────────────────────────────
// Per-ingredient tracking
// ─────────────────────────────────────────────────────────────────────────────
struct Dispense {
    char     name[24];    // ingredient name (null-terminated, from DISPENSE packet)
    uint16_t ml;          // volume in mL
    uint32_t startMs;     // millis() when this dispense began
    uint32_t durMs;       // total duration = ml * MS_PER_ML
    uint8_t  pct;         // last rendered percentage (0-100), used to skip no-op frames
    bool     active;      // slot is occupied
    bool     done;        // progress reached 100 %
};

// ─────────────────────────────────────────────────────────────────────────────
// Globals
// ─────────────────────────────────────────────────────────────────────────────
TFT_eSPI    tft;
TFT_eSprite barSpr(&tft);   // 288×14 sprite — ~8 KB, reused for every bar redraw

// Touch controller — uses the default SPI object (VSPI) remapped to the HSPI pins
// so it shares the same physical wires as TFT_eSPI (which drives HSPI itself).
// Two SPI peripherals on the same GPIOs is valid on ESP32 as long as their CS
// pins are separate and only one is asserted at a time.
static XPT2046_Touchscreen   ts(TOUCH_CS);   // TOUCH_CS=33 from build flags

char         username[32] = "";      // empty = not yet logged in
DisplayState state        = ST_IDLE;
Dispense     disp[MAX_DISP];
uint8_t      dispN        = 0;        // number of occupied slots
int16_t      cardH        = 100;      // recalculated whenever dispN changes
bool         spriteOk     = false;    // true if barSpr allocated successfully

uint32_t     completeAt   = 0;        // millis() when COMPLETE was received



// Team34 (main controller) MAC — learned dynamically on first ESP-NOW receive
static uint8_t mainMAC[6]  = {};
static bool    mainMACKnown = false;

// Layout debounce: batch rapid DISPENSE commands into one screen redraw
bool         layoutDirty  = false;
uint32_t     layoutDirtyAt = 0;
#define LAYOUT_DEBOUNCE   50UL   // ms — wait this long after last DISPENSE before redrawing

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW receive buffer
//   The callback runs on the WiFi task (core 0); the main loop runs on core 1.
//   We use a simple flag + buffer — safe enough for low-frequency text messages.
//   For production use, replace with a FreeRTOS queue.
// ─────────────────────────────────────────────────────────────────────────────
volatile bool recvFlag = false;
char          recvBuf[251];   // +1 for null terminator (ESP-NOW max payload = 250)

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void     parseCmd(const char *s);
void     drawIdle();
void     drawDispLayout();
void     drawStopButton();
void     updateBars();
void     drawComplete();
void     recalcCardH();
void     drawHeader();     
void     updateWeight(float w);                                  //added function
uint16_t blendColor(uint16_t a, uint16_t b, float t);
void     gradientFill(int x, int y, int w, int h, uint16_t topC, uint16_t botC);

// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW receive callback
//   Keep this minimal: copy data, set flag, return immediately.
//   Heavy work (display rendering) happens in loop() after the flag is detected.
//
//   Callback signature changed in arduino-esp32 v3 (ESP-IDF v5+).
//   The #if below selects the correct one at compile time.
// ─────────────────────────────────────────────────────────────────────────────
#if ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *senderMAC = info->src_addr;
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    const uint8_t *senderMAC = mac;
#endif
    // Register the main controller as a peer the first time we hear from it,
    // so we can send STOP commands back.
    if (!mainMACKnown) {
        memcpy(mainMAC, senderMAC, 6);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mainMAC, 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
        mainMACKnown = true;
        Serial.printf("[INFO] Registered main controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mainMAC[0], mainMAC[1], mainMAC[2], mainMAC[3], mainMAC[4], mainMAC[5]);
    }

    if (len <= 0 || len > 250 || recvFlag) return;  // drop if out of range or buffer busy
    memcpy(recvBuf, data, len);
    recvBuf[len] = '\0';
    recvFlag = true;  // signal loop() to process recvBuf
}

// ─────────────────────────────────────────────────────────────────────────────
// Send a command back to the main controller ESP32 via ESP-NOW
// ─────────────────────────────────────────────────────────────────────────────
static void sendToMain(const char *msg) {
    if (!mainMACKnown) { Serial.println("[WARN] Main MAC unknown — STOP not sent"); return; }
    esp_now_send(mainMAC, (const uint8_t *)msg, strlen(msg));
    Serial.printf("[MAIN] >> \"%s\"\n", msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Colour helper — linear interpolate two RGB565 values
// ─────────────────────────────────────────────────────────────────────────────
uint16_t blendColor(uint16_t a, uint16_t b, float t) {
    int r = ((a >> 11) & 0x1F) + (int)(t * (int)(((b >> 11) & 0x1F) - ((a >> 11) & 0x1F)));
    int g = ((a >>  5) & 0x3F) + (int)(t * (int)(((b >>  5) & 0x3F) - ((a >>  5) & 0x3F)));
    int bl = (a & 0x1F)        + (int)(t * (int)( (b & 0x1F)        - (a & 0x1F)));
    return ((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)bl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Gradient fill — draw a vertical gradient directly on TFT (line by line)
//   Called for backgrounds; not used for the progress bar (sprite handles that)
// ─────────────────────────────────────────────────────────────────────────────
void gradientFill(int x, int y, int w, int h, uint16_t topC, uint16_t botC) {
    for (int i = 0; i < h; i++) {
        float t = (h > 1) ? (float)i / (float)(h - 1) : 0.0f;
        tft.drawFastHLine(x, y + i, w, blendColor(topC, botC, t));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Recalculate card height based on current ingredient count
// ─────────────────────────────────────────────────────────────────────────────
void recalcCardH() {
    // Reserve STOP_BTN_H pixels at the bottom for the on-screen force-stop button
    const int usable = SCR_H - HDR_H - STOP_BTN_H;
    if (dispN == 0) { cardH = usable; return; }
    cardH = usable / dispN;
    if (cardH > 140) cardH = 140;
    if (cardH <  60) cardH =  60;
}

// ─────────────────────────────────────────────────────────────────────────────
// IDLE screen
//   Before login : "MixMate" + "Waiting for user login..."
//   After login  : "MixMate" + "Hello, <USERNAME>"
// ─────────────────────────────────────────────────────────────────────────────
void drawIdle() {
    tft.fillScreen(C_DARK_BG);

    // ── "MixMate" brand title ────────────────────────────────────────────────
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(4);                         // 26 px built-in
    tft.setTextColor(C_GRAD_TOP, C_DARK_BG);   // cyan
    tft.drawString("MixMate", SCR_W / 2, SCR_H / 2 - 28);

    // Accent line below title
    tft.drawFastHLine(SCR_W / 2 - 60, SCR_H / 2 - 8, 120, C_GRAD_TOP);

    // ── Status line ─────────────────────────────────────────────────────────
    tft.setTextFont(2);                         // 16 px built-in
    if (username[0] == '\0') {
        // No username received yet
        tft.setTextColor(C_GRAY, C_DARK_BG);
        tft.drawString("Waiting for user login...", SCR_W / 2, SCR_H / 2 + 18);
    } else {
        // Username received from main ESP32
        char greeting[52];
        snprintf(greeting, sizeof(greeting), "Hello, %s", username);
        tft.setTextColor(C_WHITE, C_DARK_BG);
        tft.drawString(greeting, SCR_W / 2, SCR_H / 2 + 18);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Dispensing layout — draw all static card elements
//   Called when dispN changes (new ingredient added or START received).
//   Dynamic elements (progress bars, %) are handled by updateBars().
// ─────────────────────────────────────────────────────────────────────────────
void drawDispLayout() {
    recalcCardH();

    tft.fillScreen(C_DARK_BG);

    // Header 
    drawHeader();

    // One card per active ingredient
    for (int i = 0; i < dispN; i++) {
        if (!disp[i].active) continue;

        int cy  = HDR_H + i * cardH;       // card top (absolute)
        int cxL = MARGIN;                   // card left

        // Card background (4 px top/bottom clearance between cards)
        tft.fillRoundRect(cxL, cy + 4, SCR_W - 2 * MARGIN, cardH - 8, 8, C_CARD_BG);

        // ── Ingredient name ──────────────────────────────────────────────
        // Offset nameY to sit inside the card with top margin
        int nameY = cy + MARGIN + (cardH >= 90 ? 14 : 10);
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(C_WHITE, C_CARD_BG);
        // Font 4 (26 px) for tall cards, Font 2 (16 px) for compact cards
        tft.setTextFont(cardH >= 90 ? 4 : 2);
        tft.drawString(disp[i].name, cxL + MARGIN, nameY);

        // ── Volume (only shown when cards are tall enough) ───────────────
        if (cardH >= 90) {
            char vstr[16];
            snprintf(vstr, sizeof(vstr), "%u mL", disp[i].ml);
            tft.setTextFont(2);   // 16 px
            tft.setTextColor(C_GRAY, C_CARD_BG);
            tft.drawString(vstr, cxL + MARGIN, nameY + 22);
        }

        // ── Progress bar track ───────────────────────────────────────────
        // barY is positioned near the card bottom, leaving 18 px for stats row below
        int barY = cy + cardH - MARGIN - 18 - BAR_H;
        tft.fillRoundRect(cxL + MARGIN, barY, BAR_W, BAR_H, BAR_H / 2, C_BAR_TRACK);
    }

    // Force bar redraw on next frame by invalidating cached pct values
    for (int i = 0; i < dispN; i++) disp[i].pct = 0xFF;

    drawStopButton();
}

// ─────────────────────────────────────────────────────────────────────────────
// Header with aligned weight indicator
// ─────────────────────────────────────────────────────────────────────────────

void drawHeader() {
    // Only called once on layout init — draws the full header
    gradientFill(0, 0, SCR_W, HDR_H, C_GRAD_TOP, C_GRAD_BOT);
    tft.setTextDatum(ML_DATUM);
    tft.setTextFont(4);
    tft.setTextColor(C_WHITE);
    tft.drawString("Dispensing...", MARGIN, HDR_H / 2 + 1);
}

void updateWeight(float w) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fml", w);

    // Erase only the number area (right side of header)
    gradientFill(SCR_W - 80, 0, 80, HDR_H, C_GRAD_TOP, C_GRAD_BOT);

    tft.setTextDatum(MR_DATUM);
    tft.setTextFont(4);
    tft.setTextColor(C_WHITE);
    tft.drawString(buf, SCR_W - MARGIN, HDR_H / 2 + 1);
    tft.setTextDatum(ML_DATUM);
}
// ─────────────────────────────────────────────────────────────────────────────
// On-screen FORCE STOP button — drawn at bottom of display during dispensing
// ─────────────────────────────────────────────────────────────────────────────
void drawStopButton() {
    const uint16_t C_RED = 0xF800;   // pure red in RGB565
    tft.fillRoundRect(MARGIN, STOP_BTN_Y + 4, SCR_W - 2 * MARGIN, STOP_BTN_H - 8, 10, C_RED);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(4);   // 26 px
    tft.setTextColor(C_WHITE, C_RED);
    tft.drawString("FORCE STOP", SCR_W / 2, STOP_BTN_Y + STOP_BTN_H / 2);
    tft.setTextDatum(ML_DATUM);   // restore default datum
}

// ─────────────────────────────────────────────────────────────────────────────
// Update progress bars — called every ~33 ms from loop()
//   Only redraws a card's dynamic region if the percentage actually changed.
// ─────────────────────────────────────────────────────────────────────────────
void updateBars() {
    uint32_t now = millis();

    for (int i = 0; i < dispN; i++) {
        if (!disp[i].active) continue;

        // ── Calculate progress fraction ──────────────────────────────────
        // startMs is set 650 ms in the future, so clamp to 0 during the padding window.
        uint32_t elapsed = (now >= disp[i].startMs) ? (now - disp[i].startMs) : 0;
        float frac = (disp[i].durMs > 0)
                     ? (float)elapsed / (float)disp[i].durMs
                     : 1.0f;
        if (frac > 1.0f) frac = 1.0f;
        uint8_t pct = (uint8_t)(frac * 100.0f);

        // Skip redraw if nothing has changed
        if (pct == disp[i].pct) continue;
        disp[i].pct = pct;
        if (pct >= 100 && !disp[i].done) disp[i].done = true;

        // ── Layout coordinates ───────────────────────────────────────────
        int cy     = HDR_H + i * cardH;
        int barX   = 2 * MARGIN;                 // absolute X of bar (16 px from left)
        int barY   = cy + cardH - MARGIN - 18 - BAR_H;
        int statsY = barY + BAR_H + 3;           // Y of the stats text row

        // ── Draw bar into sprite then push to TFT ────────────────────────
        //    Sprite size = BAR_W × BAR_H (288 × 14 = ~8 KB).
        //    Drawing into a sprite prevents the partial-render flicker you would get
        //    from painting the track then the fill directly onto the display.
        if (spriteOk) {
            barSpr.fillSprite(C_CARD_BG);   // start with card background (hides any overdraw at corners)

            // Filled rounded track
            barSpr.fillRoundRect(0, 0, BAR_W, BAR_H, BAR_H / 2, C_BAR_TRACK);

            // Accent fill — colour transitions from cyan (0%) to fuchsia (100%)
            // giving a live visual indication of how far along dispensing is
            if (pct > 0) {
                int fillW = (int)(frac * (float)BAR_W);
                if (fillW < BAR_H) fillW = BAR_H;   // min width keeps the rounded cap legible
                uint16_t fillCol = blendColor(C_GRAD_TOP, C_GRAD_BOT, frac);
                barSpr.fillRoundRect(0, 0, fillW, BAR_H, BAR_H / 2, fillCol);
            }

            barSpr.pushSprite(barX, barY);  // blit sprite to TFT in one SPI transaction
        } else {
            // Fallback: direct draw without sprite (may show brief flicker)
            tft.fillRoundRect(barX, barY, BAR_W, BAR_H, BAR_H / 2, C_BAR_TRACK);
            if (pct > 0) {
                int fillW = (int)(frac * (float)BAR_W);
                if (fillW < BAR_H) fillW = BAR_H;
                tft.fillRoundRect(barX, barY, fillW, BAR_H, BAR_H / 2,
                                  blendColor(C_GRAD_TOP, C_GRAD_BOT, frac));
            }
        }

        // ── Stats row: percentage (left) and time remaining (right) ──────
        tft.fillRect(barX, statsY, BAR_W, 16, C_CARD_BG);  // erase old stats
        tft.setTextFont(2);   // built-in 16 px font — fast to render, no heap alloc

        if (!disp[i].done) {
            // Left: "XX%"
            char pctStr[6];
            snprintf(pctStr, sizeof(pctStr), "%u%%", pct);
            tft.setTextDatum(ML_DATUM);
            tft.setTextColor(C_GRAD_TOP, C_CARD_BG);
            tft.drawString(pctStr, barX, statsY + 8);

            // Right: "XXs left"
            int32_t remMs  = (int32_t)disp[i].durMs - (int32_t)elapsed;
            if (remMs < 0) remMs = 0;
            int remSec = ((int32_t)remMs + 999) / 1000;   // ceiling division → never shows 0s while still running
            char tStr[12];
            snprintf(tStr, sizeof(tStr), "%ds left", remSec);
            tft.setTextDatum(MR_DATUM);
            tft.setTextColor(C_GRAY, C_CARD_BG);
            tft.drawString(tStr, barX + BAR_W, statsY + 8);
        } else {
            // Centred "Done!" in green
            tft.setTextDatum(MC_DATUM);
            tft.setTextColor(C_GREEN, C_CARD_BG);
            tft.drawString("Done!", barX + BAR_W / 2, statsY + 8);
        }

        tft.setTextDatum(ML_DATUM);   // restore default datum
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Complete screen
// ─────────────────────────────────────────────────────────────────────────────
void drawComplete() {
    gradientFill(0, 0, SCR_W, SCR_H, C_GRAD_TOP, C_GRAD_BOT);

    const int cw = 272, ch = 152;
    const int cx = (SCR_W - cw) / 2;
    const int cy = (SCR_H - ch) / 2;
    tft.fillRoundRect(cx, cy, cw, ch, 16, C_CARD_BG);

    // Green circle with drawn checkmark
    const int ox = SCR_W / 2, oy = cy + 48, r = 26;
    tft.fillCircle(ox, oy, r, C_GREEN);
    // Draw a thick ✓ tick (3 parallel lines to simulate stroke width)
    for (int d = 0; d <= 2; d++) {
        tft.drawLine(ox - 14, oy + d,       ox - 4,  oy + 10 + d, C_DARK_BG);
        tft.drawLine(ox -  4, oy + 10 + d,  ox + 14, oy -  8 + d, C_DARK_BG);
    }

    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(4);   // 26 px built-in
    tft.setTextColor(C_WHITE, C_CARD_BG);
    tft.drawString("Complete!", SCR_W / 2, cy + 92);

    tft.setTextFont(2);   // 16 px built-in
    tft.setTextColor(C_GRAY, C_CARD_BG);
    tft.drawString("Enjoy your drink!", SCR_W / 2, cy + 122);
}

// ─────────────────────────────────────────────────────────────────────────────
// Command parser — updates global state from a null-terminated command string
// ─────────────────────────────────────────────────────────────────────────────
void parseCmd(const char *s) {

    // ── USERNAME:<name> ──────────────────────────────────────────────────────
    if (strncmp(s, "USERNAME:", 9) == 0) {
        snprintf(username, sizeof(username), "%s", s + 9);
        // Strip any trailing whitespace / CR / LF from the BLE-style framing
        int n = (int)strlen(username);
        while (n > 0 && (username[n - 1] == '\r' || username[n - 1] == '\n' || username[n - 1] == ' '))
            username[--n] = '\0';
        if (state == ST_IDLE) drawIdle();   // refresh welcome text live
        return;
    }

    // ── START ────────────────────────────────────────────────────────────────
    if (strcmp(s, "START") == 0) {
        memset(disp, 0, sizeof(disp));
        dispN        = 0;
        layoutDirty  = false;
        state        = ST_DISPENSING;
        drawDispLayout();   // draw the (empty) dispensing frame immediately
        return;
    }

    // ── DISPENSE:<name>,<ml> ─────────────────────────────────────────────────
    if (strncmp(s, "DISPENSE:", 9) == 0) {
        if (state != ST_DISPENSING) return;  // START must arrive first
        if (dispN >= MAX_DISP) return;       // ignore overflow

        const char *p     = s + 9;
        const char *comma = strchr(p, ',');
        if (!comma) return;              // malformed packet — no comma

        int nlen = (int)(comma - p);
        if (nlen <= 0 || nlen >= (int)sizeof(disp[0].name)) return;  // bad name

        uint16_t ml = (uint16_t)atoi(comma + 1);
        if (ml == 0 || ml > 9999) return;  // sanity-check volume

        Dispense &d = disp[dispN];
        memset(&d, 0, sizeof(d));
        strncpy(d.name, p, (size_t)nlen);
        d.name[nlen] = '\0';
        d.ml      = ml;
        d.startMs = millis() + 650UL;   // 650 ms padding so bar holds at 0% while pump spins up
        d.durMs   = (uint32_t)ml * MS_PER_ML;   // e.g. 50 mL × 600 = 30 000 ms
        d.active  = true;
        d.done    = false;
        d.pct     = 0xFF;  // force first frame draw
        dispN++;

        // Debounce layout redraws: the main controller may send all ingredients
        // in quick succession.  Rather than redrawing the full screen for each
        // DISPENSE, we set a dirty flag and redraw once the burst settles.
        if (state == ST_DISPENSING) {
            layoutDirty   = true;
            layoutDirtyAt = millis();
        }
        return;
        
    }
    // ── WEIGHT:<grams> ───────────────────────────────────────────────────────────
   if (strncmp(s, "WEIGHT:", 7) == 0) {
    if (state != ST_DISPENSING) return;
       updateWeight(atof(s + 7));
    
    return;
    }

    // ── COMPLETE ─────────────────────────────────────────────────────────────
    if (strcmp(s, "COMPLETE") == 0) {
        state      = ST_COMPLETE;
        completeAt = millis();
        drawComplete();
        return;
    }

    // Unknown command — silently ignore (avoids crashing on garbage packets)
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // ── Force-stop button ─────────────────────────────────────────────────────
    pinMode(STOP_BTN_PIN, INPUT_PULLUP);   // GPIO 0 / BOOT button, active-low

    // ── TFT init ─────────────────────────────────────────────────────────────
    tft.init();
    tft.setRotation(0);       // portrait: 320 wide × 480 tall

    // ── Touch init ───────────────────────────────────────────────────────────
    // Remap default SPI (VSPI) to the same physical pins as TFT_eSPI's HSPI.
    // CS pins (TFT_CS=15, TOUCH_CS=33) ensure only one device drives the bus.
    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
    ts.begin();
    ts.setRotation(0);

#ifdef SCREEN_TEST
    // Flashes R/G/B to confirm TFT SPI and backlight are alive.
    // Once display is confirmed working, remove #define SCREEN_TEST above.
    tft.fillScreen(TFT_RED);   delay(300);
    tft.fillScreen(TFT_GREEN); delay(300);
    tft.fillScreen(TFT_BLUE);  delay(300);
#endif
    tft.fillScreen(TFT_BLACK);

    // ── Progress bar sprite ───────────────────────────────────────────────────
    // 288 × 14 pixels × 2 bytes = ~8 KB — tiny and always succeeds on ESP32
    spriteOk = (barSpr.createSprite(BAR_W, BAR_H) != nullptr);
    if (!spriteOk) Serial.println("[WARN] barSprite alloc failed — using direct TFT draw");

    // ── ESP-NOW init ──────────────────────────────────────────────────────────
    // ESP-NOW requires WiFi in station mode; we deliberately don't connect to
    // an AP so the radio is free for peer-to-peer packets only.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Print this device's MAC address to serial.
    // You need this value to configure the sender (main controller ESP32)
    // so it knows which MAC to address its ESP-NOW packets to.
    Serial.print("[INFO] LCD ESP32 MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        // Show error on screen — no point continuing without the comm link
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextFont(4);   // 26 px built-in
        tft.setTextColor(TFT_RED);
        tft.drawString("ESP-NOW FAILED", SCR_W / 2, SCR_H / 2 - 20);
        tft.setTextFont(2);   // 16 px built-in
        tft.setTextColor(TFT_WHITE);
        tft.drawString("See Serial monitor", SCR_W / 2, SCR_H / 2 + 16);
        Serial.println("[ERROR] esp_now_init() failed — halting");
        return;  // loop() will run but do nothing
    }

    // Register the receive callback.
    // Note: On ESP-IDF v5 / arduino-esp32 v3+, the callback signature changed to
    //   void cb(const esp_now_recv_info_t*, const uint8_t*, int)
    // If you see a compile error here, update the signature accordingly.
    esp_now_register_recv_cb(OnDataRecv);

    // ── Peer registration (optional for receive-only) ─────────────────────────
    // The LCD only needs to *receive*; registering the sender as a peer is only
    // required if you want to send ACKs back.  Uncomment and fill in the main
    // controller's MAC address to enable that:
    //
    //   const uint8_t mainMAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    //   esp_now_peer_info_t peer  = {};
    //   memcpy(peer.peer_addr, mainMAC, 6);
    //   peer.channel = 0;
    //   peer.encrypt = false;
    //   esp_now_add_peer(&peer);

    Serial.println("[INFO] MixMate LCD ready — waiting for data");

    // Draw the initial idle screen
    drawIdle();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop() — non-blocking main loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {

#ifdef SERIAL_DEBUG
    // Serial command input — bench testing without the main ESP32.
    // 115200 baud, "Newline" line ending in the Serial monitor.
    // Commands: USERNAME:Alex | START | DISPENSE:Vodka,50 | COMPLETE
    // Remove #define SERIAL_DEBUG above to compile this out for production.
    static char serialBuf[251];
    static uint8_t serialPos = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialPos > 0) {
                serialBuf[serialPos] = '\0';
                Serial.print("[DBG] cmd: ");
                Serial.println(serialBuf);
                parseCmd(serialBuf);
                serialPos = 0;
            }
        } else if (serialPos < 250) {
            serialBuf[serialPos++] = c;
        }
    }
#endif

    // ── Process any pending ESP-NOW message ──────────────────────────────────
    if (recvFlag) {
        recvFlag = false;           // clear before parsing so next packet isn't dropped
        parseCmd(recvBuf);
    }

    uint32_t now = millis();
    static uint32_t lastFrame = 0;

    // ── Force-stop button (GPIO 0, active-low, 50 ms debounce) ──────────────
    {
        static bool     btnReading   = HIGH;
        static bool     btnLast      = HIGH;
        static uint32_t btnDebounceT = 0;

        bool btnRaw = (bool)digitalRead(STOP_BTN_PIN);
        if (btnRaw != btnReading) {
            btnReading   = btnRaw;
            btnDebounceT = now;
        }
        if ((now - btnDebounceT) >= 50 && btnReading != btnLast) {
            btnLast = btnReading;
            if (btnLast == LOW && state == ST_DISPENSING) {
                sendToMain("STOP");
                parseCmd("COMPLETE");   // show completion screen locally
                Serial.println("[BTN] Force stop triggered");
            }
        }
    }

    // ── On-screen FORCE STOP touch button (500 ms debounce) ─────────────────
    // Accept touches only in the bottom fifth of the screen (physical Y >= 384).
    // XPT2046 raw Y can run either top→bottom (high raw = bottom) or bottom→top
    // (low raw = bottom) depending on the board.  We cover both cases by checking
    // whether the raw Y is in the HIGH fifth OR the LOW fifth of the 300–3800 range.
    //   High fifth threshold = TS_MINY + (TS_MAXX-TS_MINY)*4/5 = 2540
    //   Low  fifth threshold = TS_MINY + (TS_MAXY-TS_MINY)*1/5 = 1000
    {
        static uint32_t touchDebounce = 0;
        if (state == ST_DISPENSING && (now - touchDebounce) > 500 && ts.touched()) {
            TS_Point p = ts.getPoint();
            const int yRange    = TS_MAXY - TS_MINY;
            const int yHigh     = TS_MINY + yRange * 4 / 5;   // 2540
            const int yLow      = TS_MINY + yRange * 1 / 5;   // 1060
            Serial.printf("[TOUCH] raw=(%d,%d)  yHigh=%d  yLow=%d\n",
                          p.x, p.y, yHigh, yLow);
            if (p.y >= yHigh || p.y <= yLow) {
                touchDebounce = now;
                sendToMain("STOP");
                parseCmd("COMPLETE");
                Serial.println("[TOUCH] Force stop triggered");
            }
        }
    }

    switch (state) {

    // ── DISPENSING ────────────────────────────────────────────────────────────
    case ST_DISPENSING:
        // Apply debounced layout redraw when new ingredients arrive in bursts
        if (layoutDirty && (now - layoutDirtyAt >= LAYOUT_DEBOUNCE)) {
            layoutDirty = false;
            drawDispLayout();
        }

        // Animate progress bars at ~30 fps (33 ms per frame)
        if (now - lastFrame >= 33) {
            lastFrame = now;
            updateBars();
        }
        break;

    // ── COMPLETE ──────────────────────────────────────────────────────────────
    case ST_COMPLETE:
        // Return to idle after the completion screen has been shown long enough
        if (now - completeAt >= COMPLETE_HOLD) {
            state = ST_IDLE;
            dispN = 0;
            memset(disp, 0, sizeof(disp));
            drawIdle();
        }
        break;

    // ── IDLE ──────────────────────────────────────────────────────────────────
    case ST_IDLE:
    default:
        // Static screen — nothing to animate.
        // USERNAME packets will call drawIdle() themselves via parseCmd().
        break;
    }
}
