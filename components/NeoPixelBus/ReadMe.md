# NeoPixelBus (esp-idf component)

This is a version of [NeoPixelBus](https://github.com/Makuna/NeoPixelBus/) that can be used as an ESP-IDF component.

This component can control one wire protocol RGB and RGBW leds like APA106, SK6812, WS2811, WS2812 and WS2813 that are commonly refered to as NeoPixels and two wire protocol RGB like Lpd8806, APA102 and SK9822 commonly refered to as DotStars.

## Using this component
In your project, add this as a submodule to your components/ directory.

git submodule add https://github.com/atanisoft/NeoPixelBus.git
git submodule update --recursive --init -- NeoPixelBus
