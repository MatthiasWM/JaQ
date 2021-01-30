/*
 * SCServo.cpp
 * Ó²¼þÍ¨ÐÅ½Ó¿Ú
 * ÈÕÆÚ: 2016.8.9
 * ×÷Õß: Ì·ÐÛÀÖ
 */


#include "SCServo.h"

#include <stddef.h>


unsigned long millis()
{
	// return current time in milliseconds
	return GetTickCount();
}

SCSProtocol::SCSProtocol()
{
	Level = 1;//³ý¹ã²¥Ö¸ÁîËùÓÐÖ¸Áî·µ»ØÓ¦´ð
	End = 1;//¶æ»ú´¦ÀíÆ÷Óë¿ØÖÆ°å´¦ÀíÆ÷¶Ë½á¹¹²»Ò»ÖÂ
}

//1¸ö16Î»Êý²ð·ÖÎª2¸ö8Î»Êý
//DataLÎªµÍÎ»£¬DataHÎª¸ßÎ»
void SCSProtocol::Host2SCS(u8* DataL, u8* DataH, int Data)
{
	if (End) {
		*DataL = (Data >> 8);
		*DataH = (Data & 0xff);
	} else {
		*DataH = (Data >> 8);
		*DataL = (Data & 0xff);
	}
}

//2¸ö8Î»Êý×éºÏÎª1¸ö16Î»Êý
//DataLÎªµÍÎ»£¬DataHÎª¸ßÎ»
int SCSProtocol::SCS2Host(u8 DataL, u8 DataH)
{
	int Data;
	if (End) {
		Data = DataL;
		Data <<= 8;
		Data |= DataH;
	} else {
		Data = DataH;
		Data <<= 8;
		Data |= DataL;
	}
	return Data;
}

void SCSProtocol::writeBuf(u8 ID, u8 MemAddr, u8* nDat, u8 nLen, u8 Fun)
{
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if (nDat) {
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSCS(bBuf, 6);

	} else {
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	u8 i = 0;
	if (nDat) {
		for (i = 0; i < nLen; i++) {
			CheckSum += nDat[i];
		}
	}
	writeSCS(nDat, nLen);
	writeSCS(~CheckSum);
}

//ÆÕÍ¨Ð´Ö¸Áî
//¶æ»úID£¬MemAddrÄÚ´æ±íµØÖ·£¬Ð´ÈëÊý¾Ý£¬Ð´Èë³¤¶È
int SCSProtocol::genWrite(u8 ID, u8 MemAddr, u8* nDat, u8 nLen)
{
	flushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	return Ack(ID);
}

//Òì²½Ð´Ö¸Áî
//¶æ»úID£¬MemAddrÄÚ´æ±íµØÖ·£¬Ð´ÈëÊý¾Ý£¬Ð´Èë³¤¶È
int SCSProtocol::regWrite(u8 ID, u8 MemAddr, u8* nDat, u8 nLen)
{
	flushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	return Ack(ID);
}

//Í¬²½Ð´Ö¸Áî
//¶æ»úID[]Êý×é£¬IDNÊý×é³¤¶È£¬MemAddrÄÚ´æ±íµØÖ·£¬Ð´ÈëÊý¾Ý£¬Ð´Èë³¤¶È
void SCSProtocol::snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8* nDat, u8 nLen)
{
	u8 mesLen = ((nLen + 1) * IDN + 4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for (i = 0; i < IDN; i++) {
		writeSCS(ID[i]);
		writeSCS(nDat, nLen);
		Sum += ID[i];
		for (j = 0; j < nLen; j++) {
			Sum += nDat[j];
		}
	}
	writeSCS(~Sum);
}

int SCSProtocol::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
	flushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	return Ack(ID);
}

int SCSProtocol::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
	flushSCS();
	u8 buf[2];
	Host2SCS(buf + 0, buf + 1, wDat);
	writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
	return Ack(ID);
}

int SCSProtocol::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, P_TORQUE_ENABLE, Enable);
}

int SCSProtocol::writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun)
{
	flushSCS();
	u8 buf[6];
	Host2SCS(buf + 0, buf + 1, Position);
	Host2SCS(buf + 2, buf + 3, Time);
	Host2SCS(buf + 4, buf + 5, Speed);
	writeBuf(ID, P_GOAL_POSITION_L, buf, 6, Fun);
	return Ack(ID);
}

//Ð´Î»ÖÃÖ¸Áî
//¶æ»úID£¬PositionÎ»ÖÃ£¬Ö´ÐÐÊ±¼äTime£¬Ö´ÐÐËÙ¶ÈSpeed
int SCSProtocol::WritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
	return writePos(ID, Position, Time, Speed, INST_WRITE);
}

//Òì²½Ð´Î»ÖÃÖ¸Áî
//¶æ»úID£¬PositionÎ»ÖÃ£¬Ö´ÐÐÊ±¼äTime£¬Ö´ÐÐËÙ¶ÈSpeed
int SCSProtocol::RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
	return writePos(ID, Position, Time, Speed, INST_REG_WRITE);
}

void SCSProtocol::RegWriteAction()
{
	writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}

//Ð´Î»ÖÃÖ¸Áî
//¶æ»úID[]Êý×é£¬IDNÊý×é³¤¶È£¬PositionÎ»ÖÃ£¬Ö´ÐÐÊ±¼äTime£¬Ö´ÐÐËÙ¶ÈSpeed
void SCSProtocol::SyncWritePos(u8 ID[], u8 IDN, u16 Position, u16 Time, u16 Speed)
{
	u8 buf[6];
	Host2SCS(buf + 0, buf + 1, Position);
	Host2SCS(buf + 2, buf + 3, Time);
	Host2SCS(buf + 4, buf + 5, Speed);
	snycWrite(ID, IDN, P_GOAL_POSITION_L, buf, 6);
}

//¶ÁÖ¸Áî
//¶æ»úID£¬MemAddrÄÚ´æ±íµØÖ·£¬·µ»ØÊý¾ÝnData£¬Êý¾Ý³¤¶ÈnLen
int SCSProtocol::Read(u8 ID, u8 MemAddr, u8* nData, u8 nLen)
{
	flushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	u8 bBuf[5];
	if (readSCS(bBuf, 5) != 5) {
		return 0;
	}
	int Size = readSCS(nData, nLen);
	if (readSCS(bBuf, 1)) {
		return Size;
	}
	return 0;
}

//¶Á1×Ö½Ú£¬³¬Ê±·µ»Ø-1
int SCSProtocol::readByte(u8 ID, u8 MemAddr)
{
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if (Size != 1) {
		return -1;
	} else {
		return bDat;
	}
}

//¶Á2×Ö½Ú£¬³¬Ê±·µ»Ø-1
int SCSProtocol::readWord(u8 ID, u8 MemAddr)
{
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if (Size != 2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

//¶ÁÎ»ÖÃ£¬³¬Ê±·µ»Ø-1
int SCSProtocol::ReadPos(u8 ID)
{
	return readWord(ID, P_PRESENT_POSITION_L);
}

//¶àÈ¦¿ØÖÆÖ¸Áî
int SCSProtocol::WriteSpe(u8 ID, s16 Speed)
{
	if (Speed < 0) {
		Speed = -Speed;
		Speed |= (1 << 10);
	}
	return writeWord(ID, P_GOAL_TIME_L, Speed);
}

//¶ÁµçÑ¹£¬³¬Ê±·µ»Ø-1
int SCSProtocol::ReadVoltage(u8 ID)
{
	return readByte(ID, P_PRESENT_VOLTAGE);
}

//¶ÁÎÂ¶È£¬³¬Ê±·µ»Ø-1
int SCSProtocol::ReadTemper(u8 ID)
{
	return readByte(ID, P_PRESENT_TEMPERATURE);
}

//PingÖ¸Áî£¬·µ»Ø¶æ»úID£¬³¬Ê±·µ»Ø-1
int SCSProtocol::Ping(u8 ID)
{
	flushSCS();
	u8 bBuf[6];
	writeBuf(ID, 0, NULL, 0, INST_PING);
	int Size = readSCS(bBuf, 6);
	if (Size == 6) {
		return bBuf[2];
	} else {
		return -1;
	}
}

int SCSProtocol::wheelMode(u8 ID)
{
	u8 bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, P_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

int SCSProtocol::joinMode(u8 ID, u16 minAngle, u16 maxAngle)
{
	u8 bBuf[4];
	Host2SCS(bBuf, bBuf + 1, minAngle);
	Host2SCS(bBuf + 2, bBuf + 3, maxAngle);
	return genWrite(ID, P_MIN_ANGLE_LIMIT_L, bBuf, 4);
}

//¸´Î»¶æ»ú²ÎÊýÎªÄ¬ÈÏÖµ
int SCSProtocol::Reset(u8 ID)
{
	flushSCS();
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	return Ack(ID);
}

int	SCSProtocol::Ack(u8 ID)
{
	if (ID != 0xfe && Level) {
		u8 buf[6];
		u8 Size = readSCS(buf, 6);
		if (Size != 6) {
			return 0;
		}
	}
	return 1;
}



SCServo::SCServo()
{
	IOTimeOut = 2;
	pSerial = INVALID_HANDLE_VALUE;
}

int SCServo::open(const char* port)
{
	if (pSerial != INVALID_HANDLE_VALUE) return 0;

	// Create port
	pSerial = CreateFile("\\\\.\\COM3",
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (pSerial == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			return -2;
		}
		return -1;
	}

	SetupComm(pSerial, 1024, 1024);

	// Properties
	COMMPROP commProp = { 0 };
	GetCommProperties(pSerial, &commProp);

	// Parameters
	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(pSerial, &dcbSerialParams)) {
		return -3;
	}
	dcbSerialParams.BaudRate = 1000000; // CBR_256000;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	dcbSerialParams.fRtsControl = dcbSerialParams.fDtrControl = 0;
	if (SetCommState(pSerial, &dcbSerialParams)==0) {
		int err = GetLastError();
		return -4;
	}

	// Timeouts
	/*
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	2
		timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if (!SetCommTimeouts(hSerial, &timeouts)) {
		//error occureed. Inform user
	}
	*/

	PurgeComm(pSerial, PURGE_TXCLEAR | PURGE_RXCLEAR);

	return 0;
}

void SCServo::close()
{
	if (pSerial == INVALID_HANDLE_VALUE)
		return;
	CloseHandle(pSerial);
	pSerial = INVALID_HANDLE_VALUE;
}

int SCServo::readSCS(unsigned char* nDat, int nLen)
{
	int Size = 0;
	int ComData;
	unsigned long t_begin = millis();
	unsigned long t_user;
	while (1) {
		unsigned char buf[1] = { 0 };
		DWORD dwBytesRead = 0;
		if (!ReadFile(pSerial, buf, 1, &dwBytesRead, NULL)) {
			int x = 3;
			//error occurred. Report to user.
		}
		ComData = buf[0];
		if (ComData != -1) {
			if (nDat) {
				nDat[Size] = ComData;
			}
			Size++;
			t_begin = millis();
		}
		if (Size >= nLen) {
			break;
		}
		t_user = millis() - t_begin;
		if (t_user > IOTimeOut) {
			break;
		}
	}
	return Size;
}

int SCServo::writeSCS(unsigned char* nDat, int nLen)
{
	if (nDat == NULL) {
		return 0;
	}
	DWORD dwBytesWritten = 0;
	if (!WriteFile(pSerial, nDat, nLen, &dwBytesWritten, NULL)) {
		int x = 3;
		//error occurred. Report to user.
	}
	return nLen; // pSerial->write(&bDat, 1);
	//return -1; // pSerial->write(nDat, nLen);
}

int SCServo::writeSCS(unsigned char bDat)
{
	DWORD dwBytesWritten = 0;
	if (!WriteFile(pSerial, &bDat, 1, &dwBytesWritten, NULL)) {
		int x = 3;
		//error occurred. Report to user.
	}
	return 1; // pSerial->write(&bDat, 1);
}

void SCServo::flushSCS()
{
	// FIXME: use timeouts, etc.
	PurgeComm(pSerial, PURGE_RXCLEAR);
	return;

	DWORD dwBytesRead = 0;
	do {
		unsigned char buf[1] = { 0 };
		if (!ReadFile(pSerial, buf, 1, &dwBytesRead, NULL)) {
			int x = 3;
			//error occurred. Report to user.
		}
	} while (dwBytesRead);

	// while (pSerial->read() != -1);
}



#if 0
int CSCComm::GetComList(void)
{
	CHAR Name[25];
	UCHAR szPortName[25];
	LONG Status;
	DWORD dwIndex = 0;
	DWORD dwName;
	DWORD dwSizeofPortName;
	DWORD Type;
	HKEY hKey;
	LPCTSTR data_Set = "HARDWARE\\DEVICEMAP\\SERIALCOMM\\";
	dwName = sizeof(Name);
	dwSizeofPortName = sizeof(szPortName);

	long ret0 = RegOpenKeyEx(HKEY_LOCAL_MACHINE, data_Set, 0, KEY_READ, &hKey);//打开一个制定的注册表键,成功返回ERROR_SUCCESS即“0”值
	if (ret0 == ERROR_SUCCESS)
	{
		ComListNum = 0;
		do
		{
			Status = RegEnumValue(hKey, dwIndex++, Name, &dwName, NULL, &Type, szPortName, &dwSizeofPortName);//读取键值 
			if ((Status == ERROR_SUCCESS) || (Status == ERROR_MORE_DATA))
			{
				ComSerialList[ComListNum] = CString(szPortName);// 串口字符串保存 
				TRACE("serial:%s\n", ComSerialList[ComListNum]);
				ComListNum++;// 串口计数 
			}
			//每读取一次dwName和dwSizeofPortName都会被修改
			dwName = sizeof(Name);
			dwSizeofPortName = sizeof(szPortName);
		} while ((Status == ERROR_SUCCESS) || (Status == ERROR_MORE_DATA));

		RegCloseKey(hKey);
	}
	return ComListNum;
}

CString CSCComm::Com2DevCom(CString ComStrName)
{
	int ComNum = (USHORT)_tcstoul(ComStrName.GetBuffer() + 3, NULL, 10);
	if (ComNum > 9) {
		ComStrName.Format("\\\\.\\COM%d", ComNum);
	}
	return ComStrName;
}

CString CSCComm::Com2DevCom(int ComNum)
{
	CString ComStrName;
	if (ComNum > 9) {
		ComStrName.Format("\\\\.\\COM%d", ComNum);
	} else {
		ComStrName.Format("COM%d", ComNum);
	}
	return ComStrName;
}


char lastError[1024];
FormatMessage(
	FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
	NULL,
	GetLastError(),
	MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
	lastError,
	1024,
	NULL);
#endif


#if 0

Memory first address Function Number of bytes Initial value Storage area Permission Minimum value Maximum value Unit Value analysis
DEC HEX Low first, high last If the function address uses two bytes of data, the low byte is in the front addressand the high byte is in the back address
 0 0x00     3	Firmware major version number
 1 0x01     7	Firmware minor version number
 3 0x03     9	Servo main version number
 4 0x04     3	Servo minor version number
 5 0x05			ID: 0-253, unique identification code on the bus, no duplicate ID number on the same bus.No. 254 (OxFE)is the broadcast ID, and the broadcast does not return a response packet
 6 0x06     1	Baud rate: 0 - 7 respectively represent the baud rates as follows : 1000000, 500000, 250, 000, 128000, 115200, 76800, 57600, 38400
 7 0x07		0	Return delay: 0-254, 2us The minimum unit is 2us, and the maximum return delay can be set 254 * 2 = 508us
 8 0x08     0	Response status level: 0/1, 0 : No response packet is returned for commands other than the read commandand PING command 1 : Response packet is returned for all commands
 9 0x09     0	Minimum angle limit: 0-4094, Set the minimum limit of the movement stroke, the value is less than the maximum angle limit, the value is 0 for multi - turn absolute position control
11 0x0B  4095	Maximum angle limit: 1-4095, step Set the maximum limit of the movement stroke, the value is greater than the minimum angle limit, the value is 0 for multi - turn absolute position control
13 0x0D    70	Maximum temperature limit: 0-100°C, Maximum operating temperature limit, if set to 70, the maximum temperature is 70 degrees Celsius, and the setting accuracy is 1 degrees Celsius
14 0x0E	   80	Maximum input voltage: 0-254, 0.1V, If the maximum input voltage is set to 80, the maximum operating voltage is limited to 8.0V, and the setting accuracy is 0.1V
15 0x0F    40	Minimum input voltage: 0-254, 0.1V, If the minimum input voltage is set to 40, the minimum operating voltage is limited to 4.0V, and the setting accuracy is 0.1V
16 0x10  1000	Maximum torque: 0-1000, 0.10%, Set the maximum output torque limit of the servo, set 1000 = 100 % *locked - rotor torque, assign value to the torque limit of No. 48 address after power - on
18 0x12    12	Setting Byte: 0-254, Special function byte, can not be modified without special requirements, see special byte bit analysis for details
19 0x13    44   Protection Switch: 0-254, Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 Corresponding bit setting 1 is to open the corresponding protection Voltage Sensor Temperature Current Angle Overload The corresponding bit setting 0 is to close the corresponding protection
20 0x14    47	LED alarm condition: 0-254, Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 The corresponding bit is set to 1 to turn on the flashing alarm.Voltage, sensor, temperature, current, angle, and overload The corresponding bit is set to 0 to turn off the flashing alarm
21 0x15	   32	PID: P: 0-254, Proportional coefficient of control motor
22 0x16    32	PID: D: 0-254, Differential coefficient of control motor
23 0x17     0	PID: I:	0-254, integral coefficient of control motor
24 0x18    16	Punch: Minimum starting force: 0-1000, 0.1%, Set the minimum output starting torque of the servo, set 1000 = 100 % *locked - rotor torque
26 0x1A     1	CW Deadband: 0-32, The minimum unit is a minimum resolution angle
27 0x1B     1	CCW Deadband: 0-32, The minimum unit is a minimum resolution angle
28 0x1C   500	Overload Protection current: 0-511, 6.5mA, The maximum current that can be set is 500 * 6.5mA = 3250mA
30 0x1E     1	Angle resolution: 1-100, The amplification factor of the minimum resolution angle(degree / step) of the sensor, modify this value to expand the number of control circles
31 0x1F     0	Position correction: -2047-2047, BIT11 is the direction bit, indicating positive and negative directions, other bits can be expressed in the range of 0 - 2047 steps
33 0x21     0	Operation mode: 0-2, 0: Position servo mode 1 : Motor constant speed mode, controlled by parameter 0x2E operating speed parameter, the highest bit BIT15 is the direction bit 2 : PWM open loop speed regulation mode, using parameter 0x2C Running time parameter control, BIT11 is the direction bit 3 : Step servo mode, the parameter 0x2A target position is used to indicate the number of steps, the highest bit BIT15 is the direction bit
34 0x22    20	Protection torque: 0-254, 1.0%, Output torque after entering the overload protection, for example, setting 20 means 20 % of the maximum torque
35 0x23   200	Protection time: 0-254, 10ms, The current load output exceeds the overload torque and keeps the timing time, such as 200 means 2 seconds, the maximum can be set 2.5 seconds
36 0x24    80	Overload torque: 0-254, 1.0%, The maximum torque threshold for starting the overload protection time, if 80 is set to indicate 80 % of the maximum torque
37 0x25    10	Velocity P Gain: 0-254, In motor constant speed mode(mode 1), speed loop proportional coefficient
38 0x26   200	Overcurrent protection time: 0-254, 10ms, The maximum can be set 254 * 10ms = 2540ms
39 0x27    10	Velocity I Gain: 0-254, In motor constant speed mode(mode 1), speed loop integral coefficient
40 0x28     0	Torque switch: 0/1/128, Write 0: turn off the torque output; write 1: turn on the torque output; write 128: the current position is corrected to 2048
41 0x29     0	Acceleration: 0-254, 100 steps / s ^ 2, If set to 10, the speed will change according to the accelerationand deceleration of 1000 steps per second squared
42 0x2A     0	* Target position: -32766-32766, Each step is a minimum resolution angle, absolute position control mode, the maximum corresponds to the maximum effective angle
44 0x2C     0	Run time: 0-1000, 0.10%, Goal PWM
46 0x2E     0	Goal Velocity: 0-254 steps/s, The number of steps in the unit time(per second), 50 steps / second = 0.732 RPM(laps per minute)
48 0x30  1000	Torque limit: 0-1000, 1.0%(sic), The initial value of power - on will be assigned by the maximum torque(0x10), and the user can program to modify this value to control the maximum torque output
55 0x37     1	Lock flag: 0/1, Write 0 to close the write lock, and the value written to the EPROM address is saved when power is off Write 1 to open the write lock, and the value written to the EPROM address is not saved when power is off
56 0x38  3076	Current position: Feedback the number of steps of the current position, each step is a minimum resolution angle; absolute position control mode, the maximum value corresponds to the maximum effective angle
58 0x3A     0	Current speed: step/s, Feedback the current motor rotation speed, the number of moving steps per unit time(per second)
60 0x3C     0	Current load: 0.1%, The current control output voltage duty cycle of the driving motor
62 0x3E    72	Current voltage: 0.1V, Current servo operating voltage
63 0x3F    22	Current temperature: 1°C, The current internal working temperature of the servo
64 0x40     0	Asynchronous write flag: When using asynchronous write instructions, the flag bit
65 0x41     0	Servo state: Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 The corresponding bit is set to 1, indicating that the corresponding error occurs.Voltage, sensor, temperature, current, angle, overload, and the corresponding bit is 0 for no corresponding error.
66 0x42     0	Move flag: The flag is 1 when the servo is moving, and 0 when the servo is stopped
69 0x45     1	Present current: 6.5mA The maximum measurable current is 500 * 6.5mA = 3250mA

#endif
