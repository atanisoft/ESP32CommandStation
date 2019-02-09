# TODO list for DCC++ESP32

## For v1.2.0:

- [x] Dcc: verify and stabilize signal generation
- [x] Dcc: verify programming track operation
- [x] Doc: document configuration options further
- [x] Web: drop websocket from web interface (html only) as it is not used today.
- [ ] Web: test web interface end-to-end
- [x] Web: test jmri connection to ensure it still works as expected
- [ ] Misc: step through InfoScreen code to make sure it is working as expected, there are a few bugs in here now it would seem as the screen doesn't always
update when it should.
- [x] Misc: extend the InfoScreen rotating line to display LCC details, it is a bit hacky right now to add new details.
- [ ] Compile: verify pre-processor checks are all correct

### Bugs in JMRI
- [x] JMRI interprets <H ID ADDR IDX STATE> as <H ID STATE>. PR submitted https://github.com/JMRI/JMRI/pull/6537

## After v1.2.0:

- [ ] Nextion: add notification of turnout state change when changed external to the nextion code
- [ ] Web: investigate tcp/ip hang
- [ ] Web: auto-refresh of status pages
- [ ] Doc: write up arduino IDE build instructions
- [ ] Web: add busy/wait spinner for when data is loading (or being refreshed) in the web interface
- [ ] Dcc: add support for RailCom cut-out. This is a lower priority item as I still need to read up on it further, but in short the dcc signal goes OFF
(dcc output to both rails is LOW) for a set duration where RailCom packets are transmitted over the rails to a decoder board and then dcc signal
resumes. This will be a long-term requirement for LCC integration, I also need a couple RailCom capable dcc decoders (might already have a couple,
not sure)
- [ ] Dcc: fix signal generation so it doesn't crash up when spi_flash disables cache. This is the hardest and biggest issue by far and needs to be
fixed somehow but I haven't found a working solution yet. It will very likely require a ground up re-write with streaming packet data to the ISR.
- [ ] Lcc: adjust InfoScreen LCC details so they are actually useful, right now it is a placeholder.
- [ ] Nextion: replace Routes page with a Setup page which will include route creation
- [ ] Web: SoftAP support (https://github.com/atanisoft/DCCppESP32/issues/4)