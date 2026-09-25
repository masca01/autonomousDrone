import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from plot_live import Lines, parse_line

class TelemetryTests(unittest.TestCase):
    def test_pid(self):
        line='ROLL  angle=+20.0 deg rate=+0.0 deg/s error=-20.0 P=-0.400 I=+0.000 D=+0.000 correction=-0.400'
        self.assertEqual(parse_line(line),[('roll',20,0,-0.4)])
        self.assertEqual(parse_line(line.replace('ROLL ','PITCH')), [('pitch',20,0,-0.4)])
    def test_attitude(self):
        line='roll=+10.0 deg | pitch=-2.0 deg | yaw_relative=+30.0 deg | gyro_corrected[deg/s] X=+1.20 Y=-0.20 Z=+0.00'
        self.assertEqual(parse_line(line),[('roll',10,1.2,None),('pitch',-2,-0.2,None)])
    def test_ignore(self):
        for line in ['READY: tilt now','Keep STILL: calibration 1/500 samples','n=42 | accel[g] X=1','ROLL angle=nan','corrupted']:
            self.assertEqual(parse_line(line),[])
    def test_chunks(self):
        stream=Lines()
        self.assertEqual(stream.feed(b'REA'),[])
        self.assertEqual(stream.feed(b'DY\r\nROLL\nPIT'),['READY','ROLL'])
        self.assertEqual(stream.feed(b'CH\n'),['PITCH'])
        stream.feed(b'x'*9000); self.assertEqual(stream.pending,b'')

if __name__=='__main__': unittest.main()
