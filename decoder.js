function Decoder(bytes, port) {
    var decoded = {};
    decoded.bat = ((bytes[0] << 8) + bytes[1]) / 100;
    decoded.boot_nb = ((bytes[2] << 8) + bytes[3]) ;
    return decoded;
}
