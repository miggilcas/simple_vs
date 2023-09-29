import time, sys
from pymavlink import mavutil
from colorama import init, Fore
import argparse
import socket
import glob, serial
import signal

init()

parser = argparse.ArgumentParser(description = 'MAVLink reader.')
parser.add_argument('-s', metavar = 'source', help = 'Input stream source. Can be a physical port or a socket address. Example: COM3, udp:127.0.0.1:14550')
parser.add_argument('-b', metavar = 'baudrate', help = 'Baudrate')
parser.add_argument('-f', metavar = 'filter', help = 'Shows only specified message type')
parser.add_argument('-da', metavar = 'destination_address', help = 'Sends packets to specified address')
parser.add_argument('-dp', metavar = 'destination_port', help = 'Sends packets to specified port')
parser.add_argument('-so', metavar = 'serial_out', help = 'Sends packets (plain text) to specified serial port')

total_corrupted = 0
n_corrupted = 0
start_time = time.time()

def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range (256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def main(port, baudrate, filt, dest_a, dest_p, serial_out):
    global total_corrupted, n_corrupted

    print("Reading MAVLink messages on port %s (%s Bd)" % (port, baudrate))
    if filt:
        print("Filtering %s* messages" % filt)

    sock = None
    if dest_a and dest_p:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Forwarding messages to %s:%d" % (dest_a, int(dest_p)))

    out_port = None
    if serial_out:
        try:
            out_port = serial.Serial(port = serial_out,
                                     baudrate = baudrate,
                                     bytesize = serial.EIGHTBITS,
                                     parity = serial.PARITY_NONE,
                                     stopbits = serial.STOPBITS_ONE,
                                     timeout = 0.5)
            print("Forwarding messages to %s (%d baud)" % (serial_out, int(baudrate)))
        except Exception as e:
            print("%sCould not open serial port %s.\n%s\nSkipping serial forwarding.%s" % (Fore.RED, serial_out, e, Fore.RESET))


    # mavutil.mavlink.WIRE_PROTOCOL_VERSION = '2.0'
    mavutil.os.environ['MAVLINK20'] = '2'
    try:
        decoder = mavutil.mavlink_connection(port, baud = baudrate)
    except (OSError, serial.SerialException):
        print("%sCould not open port %s.%s" % (Fore.RED, port, Fore.RESET))
        print("Ports available:", serial_ports())
        sys.exit(0)
    decoder.mav.request_data_stream_send(decoder.target_system, decoder.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

    last_timestamp = time.time()
    signal.signal(signal.SIGINT, signal_handler)
    corrupted = 0
    while True:
        msg = decoder.recv_match(blocking = False)
        if msg:
            if msg.get_type().startswith("BAD_DATA") or msg.get_type().startswith("UNKNOWN"):
                corrupted += 1
            else:
                last_timestamp = time.time()
                now = time.localtime()
                if corrupted > 0:
                    print("[%02d:%02d:%02d] %sCorrupted messages in between: %d.%s" % (now.tm_hour, now.tm_min, now.tm_sec, Fore.RED, corrupted, Fore.RESET))
                    total_corrupted += corrupted
                    n_corrupted += 1
                    corrupted = 0
                if not filt or msg.get_type().startswith(filt):
                    print("[%02d:%02d:%02d] %s[sys: %d comp: %d]%s %s%s%s" % (now.tm_hour, now.tm_min, now.tm_sec, Fore.CYAN, msg.get_srcSystem(), msg.get_srcComponent(), Fore.RESET, Fore.YELLOW, msg, Fore.RESET))
                    print(msg.to_dict() )
                    if sock:
                        sock.sendto(str(msg).encode('utf-8'), (dest_a, int(dest_p)))
                    if out_port:
                        string = "[%02d:%02d:%02d] [sys: %d comp: %d] %s" % (now.tm_hour, now.tm_min, now.tm_sec, msg.get_srcSystem(), msg.get_srcComponent(), msg)
                        out_port.write(string.encode('utf-8'))
                
        if time.time() - last_timestamp > 5: # Reconectamos si llevamos 5 segundos sin recibir mensajes
            last_timestamp = time.time()
            decoder = mavutil.mavlink_connection(port, baud = baudrate)
            decoder.mav.request_data_stream_send(decoder.target_system, decoder.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
            print("Reconnecting to port %s (%s Bd)" % (port, baudrate))

def signal_handler(sig, frame):
    global total_corrupted, n_corrupted, start_time

    print("\n[%02d:%02d:%02d] %sAverage corrupted messages in between: %d.%s" % (time.localtime().tm_hour, time.localtime().tm_min, time.localtime().tm_sec, Fore.WHITE, round(total_corrupted / n_corrupted), Fore.RESET))
    print("[%02d:%02d:%02d] %sAverage corrupted messages per second: %d.%s" % (time.localtime().tm_hour, time.localtime().tm_min, time.localtime().tm_sec, Fore.WHITE, round(total_corrupted / (time.time() - start_time)), Fore.RESET))
                    
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    
    args = parser.parse_args()
    if args.s:
        port = args.s
    else:
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin') or sys.platform.startswith('darwin'):
            port = '/dev/ttyS3'
        else:
            port = 'COM3'

    if args.b:
        baudrate = args.b
    else:
        baudrate = 57600

    main(port, baudrate, args.f, args.da, args.dp, args.so)