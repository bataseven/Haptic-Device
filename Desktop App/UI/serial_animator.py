from cmath import inf
import os
import serial
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import rcParams
import matplotx
import collections
import time
import glob
import re
import threading
from matplotlib.widgets import CheckButtons
import logging
import argparse

logging.getLogger('matplotlib').setLevel(level=logging.CRITICAL)

rcParams.update({'figure.autolayout': True})
plt.style.use(matplotx.styles.github["dimmed"])


def clearConsole(): return os.system(
    'cls' if os.name in ('nt', 'dos') else 'clear')

parser = argparse.ArgumentParser(description='Serial Animator')
parser.add_argument("--port", type=str, help="Serial port to use")
parser.add_argument("--baud", type=int, help="Baud rate to use")

args = parser.parse_args()

args_port = args.port
args_baud = args.baud

class SerialAnimator:
    def __init__(self):
        clearConsole()
        self.serial_port = None
        self.stop_requested = False
        count = 0
        while self.serial_port is None:
            self.connect_to_serial_port()
            if self.serial_port is None:
                print("Retrying", end="")
                count += 1
                count %= 3
                print("." * (count + 1) + " " *
                      (3 - count) + "(Press Ctrl+C to exit)")
                time.sleep(1)
                clearConsole()
        self.slide_limit = 100
        self.data_count = 0
        self.last_time = 0
        self.delta_times = collections.deque(maxlen=15)
        self.delta_times.append(1) # Avoid division by zero
        self.average_rate = 0

        # Dictionary of all the data, with the label as the key.
        # Values of the dictionary are a list.
        # First element of the list is a deque, where the data is stored in (data_count, value) pairs.
        # The second element of the list is the line to be plotted.
        self.labeled_data = {}
        self.min_y = -1
        self.max_y = 1

    def connect_to_serial_port(self):
        ports = self.list_available_ports()
        for i, port in enumerate(ports):
            print("[{}] - {}".format(i, port))
        if len(ports) == 0:
            print("No serial port found.", end=" ")
            return None, None
        elif args_port is not None:
            port = args_port
            print("Using port: {}".format(port))
        elif len(ports) == 1:
            port = ports[0]
            print("Using port: {}".format(port))
        else:
            selected_port = input("Select a port: ")
            try:
                port = ports[int(selected_port)]
            except:
                print("Invalid port selected.")
                port = ports[0]
            finally:
                print("Using port: {}".format(port))
        if args_baud is not None:
            baudrate = args_baud
            print(f"Using baudrate: {baudrate}")
        else:
            baudrate = input("Enter the baudrate (Enter for 115200): ")
        if baudrate == "":
            baudrate = 115200
        else:
            try:
                baudrate = int(baudrate)
            except ValueError:
                print("Invalid baudrate. Using 115200.")
                baudrate = 115200
        try:
            self.serial_port = serial.Serial(port, baudrate)
            print("Connected to port: {}".format(port))
        except:
            print("Error: Could not open serial port. Exiting.")
            exit()

    def list_available_ports(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
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

    def read_serial_port(self):
        while True and not self.stop_requested:
            new_line = self.serial_port.readline()
            new_line = new_line.decode("utf-8")
            new_line = new_line.strip()
            self.parse_line(new_line)
            self.data_count += 1
            
            average_times = sum(self.delta_times) / len(self.delta_times)
            if average_times > 0:
                self.average_rate = 1 / average_times
            
            current_time = time.time()
            self.delta_times.append(current_time - self.last_time)
            self.last_time = current_time

    def start_reading(self):
        self.reading_thread = threading.Thread(target=self.read_serial_port)
        self.reading_thread.start()

    def parse_line(self, new_line):
        seperators = ['=', ':', ',']
        backslashed_seperators = ''.join(['\\' + ch for ch in seperators])
        expression = f'([A-z0-9_ ]+ *[{backslashed_seperators}] *[+-]?([0-9]*[.])?[0-9]+)'
        for item in re.finditer(expression, new_line):
            start_index = item.start()
            end_index = item.end()
            match = new_line[start_index: end_index]
            for ch in seperators:
                if ch in match:
                    split_match = [word.strip() for word in match.split(ch)]

            label = split_match[0]
            value = split_match[1]

            for key in self.labeled_data.keys():
                if label.capitalize() == key.capitalize():
                    self.labeled_data[key][0].append(
                        (self.data_count, float(value)))
                    break
            else:
                self.labeled_data[label] = []
                self.labeled_data[label].append(collections.deque(
                    [(self.data_count, float(value))], maxlen=self.slide_limit))
                line, = self.ax[1].plot(*zip(*self.labeled_data[label][0]))
                line.set_label(label)
                self.labeled_data[label].append(line)
                self.draw_checkbox()

            # Adjust the y-axis max and min values only if the line is visible
            if self.labeled_data[label][1].get_visible():
                if float(value) > self.max_y:
                    self.max_y = float(value)
                    self.ax[1].set_ylim(self.min_y - 0.1 * abs(self.min_y), self.max_y + 0.1 * abs(self.max_y))
                if float(value) < self.min_y:
                    self.min_y = float(value)
                    self.ax[1].set_ylim(self.min_y - 0.1 * abs(self.min_y), self.max_y + 0.1 * abs(self.max_y))

    def draw_checkbox(self):
        self.ax[0].clear()
        self.checkbox = CheckButtons(
            self.ax[0], [*self.labeled_data.keys()], [v[1].get_visible() for _, v in self.labeled_data.items()])

        self.checkbox.on_clicked(self.set_visibility)
        checkbox_count = len(self.labeled_data.keys())
        for idx, rect in enumerate(self.checkbox.rectangles):
            height = 0.08
            width = 0.4
            color = self.labeled_data[self.checkbox.labels[idx].get_text(
            )][1].get_color()

            rect.set_width(width)
            rect.set_height(height)

            aperture = 0.02

            # Center the rectangles along the vertical axis
            rect.set_xy((0, 0.5 - checkbox_count * height / 2
                         - (checkbox_count - 1) / 2 * aperture
                         + idx * height
                         + idx * aperture))

            rect.set_edgecolor(color)

            xy = rect.get_xy()
            self.checkbox.lines[idx][0].set_xdata(
                [xy[0], xy[0] + width])
            self.checkbox.lines[idx][0].set_ydata(
                [xy[1], xy[1] + height])

            self.checkbox.lines[idx][1].set_xdata(
                [xy[0], xy[0] + width])
            self.checkbox.lines[idx][1].set_ydata(
                [xy[1] + height, xy[1]])

            self.checkbox.lines[idx][0].set_color(color)
            self.checkbox.lines[idx][1].set_color(color)

            self.checkbox.labels[idx].set_position(
                (xy[0] + width + 0.05, xy[1] + height / 2))
            self.checkbox.labels[idx].set_color(color)

    def initilize_plot(self):
        self.start_reading()
        self.start_time = time.time()
        self.ax[1].grid(True, which='both')
        self.ax[1].set_xlabel('Data Points')
        self.ax[1].set_ylabel('Value')
        self.ax[1].set_ylim(self.max_y, self.min_y)
        self.ax[0].spines['top'].set_visible(False)
        self.ax[0].spines['right'].set_visible(False)
        self.ax[0].spines['bottom'].set_visible(False)
        self.ax[0].spines['left'].set_visible(False)
        self.ax[0].tick_params(
            axis='both', which='both', bottom=False, top=False, labelbottom=False, right=False, left=False, labelleft=False)

    def start_animation(self):
        self.fig, self.ax = plt.subplots(
            ncols=2, nrows=1, gridspec_kw={'width_ratios': [1, 8]})
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=1, init_func=self.initilize_plot)
        self.fig.canvas.manager.set_window_title(
            'Serial Plotter - {}'.format(self.serial_port.port))
        plt.show()

    def set_visibility(self, label):
        self.labeled_data[label][1].set_visible(
            not self.labeled_data[label][1].get_visible())

        self.max_y = -inf
        self.min_y = inf

        for key, value in self.labeled_data.items():
            if not value[1].get_visible():
                value[1].set_label('')
            else:
                value[1].set_label("{}".format(key))
                # Re-adjust the y-axis limits according to visible lines
                for point in value[0]:
                    if point[1] > self.max_y:
                        self.max_y = point[1]
                    if point[1] < self.min_y:
                        self.min_y = point[1]

        if self.min_y == 0 or self.min_y == inf:
            self.min_y = -1
        if self.max_y == 0 or self.max_y == -inf:
            self.max_y = 1

        self.ax[1].set_ylim(self.min_y - 0.1 * abs(self.min_y), self.max_y + 0.1 * abs(self.max_y))

    def update_plot(self, frame):
        for key in self.labeled_data.keys():
            self.labeled_data[key][1].set_data(
                *zip(*self.labeled_data[key][0]))

        self.ax[1].legend(loc='upper left')

        if self.data_count < self.slide_limit:
            self.ax[1].set_xlim(0, self.data_count + 1)
        else:
            self.ax[1].set_xlim(
                self.data_count - self.slide_limit, self.data_count)

        plot_title = f"Serial Data over {self.serial_port.port}\n{time.time() - self.start_time:.1f} seconds"
        #plot_title = f"Serial Data over {self.serial_port.port}\n{time.time() - self.start_time:.1f} seconds\n{self.average_rate:.1f} Hz"
        self.ax[1].set_title(plot_title)

    def close(self):
        self.stop_requested = True
        self.serial_port.flush()
        self.serial_port.close()
        self.reading_thread.join()
        plt.close()


def main():
    serial_animator = SerialAnimator()
    serial_animator.start_animation()
    serial_animator.close()
    print("\nExiting...\n")


if __name__ == '__main__':
    main()
