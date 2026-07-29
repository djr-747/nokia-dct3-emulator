// Build a Nokia OTA browser-settings message — the "Settings received, Save?"
// binary SMS operators used to provision WAP profiles over the air.
//
// Plain script on purpose: both web runners load it with a <script> tag and
// tools/otasettings.mjs imports it for the CLI — everyone reads
// globalThis.DCT3_OTA. Pure computation, no I/O, so the pages need no server
// endpoint to build a settings message.
//
// The output is the SMS *user data* minus the header: the injector supplies the
// port-addressing UDH itself (dest 49999, src 49154), so this builds the WSP
// push PDU and the WBXML settings document.
//
// Format per the Nokia Over The Air Settings Specification, cross-checked
// against Kannel's gw/ota_prov.c and ota_prov_attr.h, which is the reference
// implementation. Structure:
//
//   WSP push:  01 06 <hdrlen> 1F <vallen> "application/x-wap-prov.browser-settings" 00 81 EA
//   WBXML:     01 01 6A 00   (version 1.1, public id "unknown", UTF-8, no string table)
//   document:  CHARACTERISTIC-LIST > CHARACTERISTIC TYPE=ADDRESS { PARM* }
//                                  > CHARACTERISTIC TYPE=URL VALUE=<homepage>
//                                  > CHARACTERISTIC TYPE=NAME { PARM NAME=<label> }
(function () {
  // Global WBXML tokens
  var END_STR_I = 0x00, END = 0x01, STR_I = 0x03;
  // Element tokens (|0x40 = has content, |0x80 = has attributes)
  var CHARACTERISTIC_LIST = 0x05, CHARACTERISTIC = 0x06, PARM = 0x07;
  var VALUE = 0x11;
  // CHARACTERISTIC TYPE= attribute-start tokens
  var TYPE_ADDRESS = 0x06, TYPE_URL = 0x07, TYPE_NAME = 0x08;
  // PARM NAME= tokens and their value tokens
  var NAME_BEARER = 0x12, VALUE_GSM_CSD = 0x45;
  var NAME_PROXY = 0x13, NAME_PROXY_TYPE = 0x16, VALUE_IPV4 = 0x77;
  var NAME_PORT = 0x14, VALUE_PORT_9201 = 0x61;
  var NAME_NAME = 0x15;
  var NAME_CSD_DIALSTRING = 0x21;
  var NAME_PPP_AUTHTYPE = 0x22, VALUE_AUTH_PAP = 0x70;
  var NAME_CSD_CALLTYPE = 0x28, VALUE_CONN_ANALOGUE = 0x72, VALUE_CONN_ISDN = 0x73;
  var NAME_CSD_CALLSPEED = 0x29, VALUE_SPEED_9600 = 0x6b, VALUE_SPEED_14400 = 0x6c;

  var MIME = 'application/x-wap-prov.browser-settings';

  // latin1 byte values — matches Buffer.from(s, 'latin1') for every code
  // point <= 0xFF, which is all these fields may carry on the wire anyway.
  function latin1(s) {
    var out = [];
    for (var i = 0; i < s.length; i++) out.push(s.charCodeAt(i) & 0xff);
    return out;
  }
  function inlineStr(s) { return [STR_I].concat(latin1(s), [END_STR_I]); }
  // <PARM NAME=.. VALUE=..> with a literal string value
  function parmStr(nameTok, s) { return [PARM | 0x80, nameTok, VALUE].concat(inlineStr(s), [END]); }
  // <PARM NAME=.. VALUE=..> where the value is itself a well-known token
  function parmTok(nameTok, valTok) { return [PARM | 0x80, nameTok, valTok, END]; }

  function buildSettings(opts) {
    opts = opts || {};
    var url = opts.url, proxy = opts.proxy, dial = opts.dial, name = opts.name;
    var speed = opts.speed || 0;
    var isdn = opts.isdn !== false;
    var auth = opts.auth || '';

    var doc = [];
    doc.push(CHARACTERISTIC_LIST | 0x40);

    // The bearer block: how to get onto the network at all.
    doc.push(CHARACTERISTIC | 0x80 | 0x40, TYPE_ADDRESS, END);
    doc = doc.concat(parmTok(NAME_BEARER, VALUE_GSM_CSD));
    if (proxy) {
      doc = doc.concat(parmStr(NAME_PROXY, proxy));
      doc = doc.concat(parmTok(NAME_PROXY_TYPE, VALUE_IPV4));
    }
    doc = doc.concat(parmTok(NAME_PORT, VALUE_PORT_9201));   // connection-oriented WSP
    if (dial) doc = doc.concat(parmStr(NAME_CSD_DIALSTRING, dial));
    // PPP_AUTHTYPE and CSD_CALLSPEED are omitted when the spec's own default is
    // already what we want: PAP is the default auth type, and the default speed
    // is 9600 when the call type is ISDN. Every omitted PARM buys 4 bytes, and
    // the whole document has to fit one 133-byte SMS — concatenation is a known
    // broken path. Pass auth/speed explicitly to force them onto the wire.
    if (auth && auth !== 'pap') doc = doc.concat(parmTok(NAME_PPP_AUTHTYPE, VALUE_AUTH_PAP));
    // ISDN, not analogue. MEASURED: with ANALOGUE the profile arrives intact but
    // the call does not work until the user switches Data call type by hand. Our
    // network assigns a UDI (unrestricted digital) traffic channel — the ISDN
    // style bearer — whereas ANALOGUE means a modem pool reached through the
    // PSTN, which nothing here offers. Pass isdn:false for a real analogue peer.
    doc = doc.concat(parmTok(NAME_CSD_CALLTYPE, isdn ? VALUE_CONN_ISDN : VALUE_CONN_ANALOGUE));
    if (speed) doc = doc.concat(parmTok(NAME_CSD_CALLSPEED,
                        speed === 14400 ? VALUE_SPEED_14400 : VALUE_SPEED_9600));
    doc.push(END);                                           // </CHARACTERISTIC>

    // The homepage. TYPE=URL carries its value as an attribute, not content.
    if (url) doc = doc.concat([CHARACTERISTIC | 0x80, TYPE_URL, VALUE], inlineStr(url), [END]);

    // The user-visible name of the settings set.
    if (name) {
      doc.push(CHARACTERISTIC | 0x80 | 0x40, TYPE_NAME, END);
      doc = doc.concat(parmStr(NAME_NAME, name));
      doc.push(END);
    }

    doc.push(END);                                           // </CHARACTERISTIC-LIST>

    var wbxml = [0x01, 0x01, 0x6a, 0x00].concat(doc);        // ver 1.1, public id 1, UTF-8, no table
    // WSP connectionless push: TID, PDU type 0x06, then the headers block —
    // a length-quoted Content-Type plus the well-known UTF-8 charset parameter.
    var ct = [0x1f, MIME.length + 3].concat(latin1(MIME), [0x00, 0x81, 0xea]);
    return new Uint8Array([0x01, 0x06, ct.length].concat(ct, wbxml));
  }

  // One SMS holds 140 octets of user data, 7 of which the port-addressing
  // header takes. Concatenation exists but is a known broken path, so a
  // settings message has to fit what is left.
  var SMS_ROOM = 140 - 7;

  // Build settings that fit one SMS, shortening the connection name if that is
  // what it takes — the name is the only cosmetic field, everything else changes
  // how the phone connects. Reports what it had to do rather than silently
  // truncating. Returns {body, name, fits}.
  function buildSettingsFitted(opts, room) {
    opts = opts || {};
    room = room || SMS_ROOM;
    var name = opts.name || '';
    for (;;) {
      var merged = {}; for (var k in opts) merged[k] = opts[k]; merged.name = name;
      var body = buildSettings(merged);
      if (body.length <= room || !name) return { body: body, name: name, fits: body.length <= room };
      name = name.slice(0, -1).replace(/\s+$/, '');          // drop a character and retry
    }
  }

  globalThis.DCT3_OTA = {
    buildSettings: buildSettings,
    buildSettingsFitted: buildSettingsFitted,
    SMS_ROOM: SMS_ROOM,
  };
})();
