import tkinter as tk
import serial
import binascii
import time
import sys
import serial.tools.list_ports
import tkinter.filedialog
from tkinter import messagebox, ttk

PAD_X = 2
PAD_Y = 2

PAGE_SIZE = 1024
CAN_ID = 0x78E

CRC_TABLE = (0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9,
             0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
             0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
             0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD)

def dword(value):
    return value & 0xFFFFFFFF

def crc32_fast(crc, data):
    crc, data = dword(crc), dword(data)
    crc ^= data
    for _ in range(8):
        crc = dword(crc << 4) ^ CRC_TABLE[crc >> 28]
    return crc

def crc32_fast_block(crc, buffer):
    for data in buffer:
        crc = crc32_fast(crc, data)
    return crc

def crc32_fast_bytes(crc, bytes_data):
    if len(bytes_data) & 3:
        raise ValueError('bytes_data length must be multiple of four')
    for index in range(0, len(bytes_data), 4):
        data = int.from_bytes(bytes_data[index : index + 4], 'little')
        crc = crc32_fast(crc, data)
    return crc

class Page(object):
    def __init__(self):
        self.crc = 0
        self.data = None
    
class CANHacker(object):
    def __init__(self, port):
        self.port = port
        
    def sendMessage(self, id, bytes):
        cmd = b"t%03X%X%s\r" % (id, len(bytes), binascii.hexlify(bytes))
        self.port.write(cmd)

class CANProgrammer(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.title("CANBoot Programmer")

        # Row 0
        self.lblPort = ttk.Label(text="Port", width=5)
        self.lblPort.grid(row=0, column=0, padx=PAD_X, pady=PAD_Y)
        
        self.cbPort = ttk.Combobox(values=list(map(lambda x: x.device, serial.tools.list_ports.comports())), width=8, state='readonly')
        self.cbPort.grid(row=0, column=1, padx=PAD_X, pady=PAD_Y)
        self.cbPort.current(0)
        
        self.lblHW = ttk.Label(text="Hardware", width=10)
        self.lblHW.grid(row=0, column=2, padx=PAD_X, pady=PAD_Y)
        
        self.cbHW = ttk.Combobox(values=["CANHacker"], width=15, state='readonly')
        self.cbHW.grid(row=0, column=3, padx=PAD_X, pady=PAD_Y)
        self.cbHW.current(0)
        
        self.lblSpeed = ttk.Label(text="Speed", width=8)
        self.lblSpeed.grid(row=0, column=4, padx=PAD_X, pady=PAD_Y)
        
        self.cbSpeed = ttk.Combobox(values=["115200"], width=12, state='readonly')
        self.cbSpeed.grid(row=0, column=5, padx=PAD_X, pady=PAD_Y)
        self.cbSpeed.current(0)
        
        # Row 1
        self.progress = ttk.Progressbar(self, orient="horizontal", length=250, mode="determinate")
        self.progress.grid(row=1, column=0, columnspan=4, padx=PAD_X, pady=PAD_Y)

        self.fsel = ttk.Button(text="...", command=self.choose_file, width=3)
        self.fsel.grid(row=1, column=4, padx=PAD_X, pady=PAD_Y)

        self.start = ttk.Button(text="Start", command=self.start, state="disabled")
        self.start.grid(row=1, column=5, padx=PAD_X, pady=PAD_Y)
        
        # Row 2
        self.status = tk.StringVar()        
        self.lblStatus = tk.Label(self, bd=1, relief=tk.SUNKEN, anchor=tk.W, textvariable=self.status, width=60)
        self.lblStatus.grid(row=2, column=0, columnspan=6)
        self.status.set('Choose file to flash')
        
        self.bytes = 0
        self.maxbytes = 0

    def choose_file(self):
        self.file = tkinter.filedialog.askopenfile(filetypes=[("Firmware files","*.bin")], mode="rb")
        if self.file is not None:
            self.pages = []

            while True:
                page = Page()
                page.data = self.file.read(PAGE_SIZE)
                if not page.data:
                    break
                if len(page.data) < PAGE_SIZE:
                    page.data = page.data.ljust(PAGE_SIZE, b'\xFF')
                
                page.crc = crc32_fast_bytes(0xFFFFFFFF, page.data)
                self.pages.append(page)

            self.maxbytes = len(self.pages) * PAGE_SIZE / 8
            self.progress["value"] = 0
            self.progress["maximum"] = self.maxbytes

            self.status.set("Ready to program, %d page(s) %d byte(s)" % (len(self.pages), len(self.pages) * PAGE_SIZE))
            self.start.config(state="")
            
    def enterBoot(self):
        self.status.set("Entering bootloader mode... %d" % self.enterBootCounter)

        self.enterBootCounter -= 1
        self.hw.sendMessage(CAN_ID, bytes.fromhex("01"))
        if self.enterBootCounter > 0:
            self.after(250, self.enterBoot)
        else:
            self.programDevice()
            
    def finished(self):
        self.port.close()
        self.start.config(state="")
        self.fsel.config(state="")
        self.status.set("Programming finished")

    def programDevice(self):
        self.bytes = 0

        for idx, page in enumerate(self.pages):
            hdr = b'\x02' + idx.to_bytes(1, 'little') + page.crc.to_bytes(4, 'little')
            self.hw.sendMessage(CAN_ID, hdr)
            
            for i in range(0, int(PAGE_SIZE / 8)):
                chunk = page.data[8 * i : 8 * i + 8]
                self.hw.sendMessage(CAN_ID, chunk)
                self.bytes += 1
                self.progress["value"] = self.bytes
                self.status.set("Sending page %d (%d of %d)" % (idx, i * 8 + 8, PAGE_SIZE))
                time.sleep(0.005)
                self.update()

            self.status.set("Wait to program page %d" % idx)
            self.update()
            time.sleep(0.5)

        self.finished()

    def start(self):
        try:
            self.port = serial.Serial(self.cbPort.get(), int(self.cbSpeed.get()), timeout=5)
            self.hw = getattr(sys.modules[__name__], self.cbHW.get())(self.port)
            
            self.start.config(state="disabled")
            self.fsel.config(state="disabled")
        except Exception as ex:
            messagebox.showerror("Error" , "Failed to open port: %s" % str(ex))
            return
        
        self.enterBootCounter = 10
        self.after(250, self.enterBoot)

app = CANProgrammer()
app.mainloop()
