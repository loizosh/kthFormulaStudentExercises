import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
import matplotlib.pyplot as plt
import tkinter as tk
from numpy import linspace # Used for the GUI environment
from sympy import var # Used to accept mathematical expressions as user input
from sympy import sympify # Used to accept mathematical expressions as user input
from sympy import Symbol # Used to accept mathematical expressions as user input
from sympy import latex # Used to render TeX characters
plt.rcParams['text.usetex'] = True # Used to render TeX characters


class WindowManagement:
    def __init__(self, window, window_name):
        # Initialize window
        self.window = window
        self.window_name = window_name
        self.window.title(self.window_name) # Give name to the window
        self.window.geometry('600x550') # Dimensions of window
        self.window.bind('<Escape>', lambda e: self.destroy_window())
        self.initialize_entry_boxes()
        self.initialize_buttons()

        # For plotting
        self.fig = Figure(figsize=(6,6))      
        self.plot_object = self.fig.add_subplot(111)   
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)            
        self.canvas.get_tk_widget().pack()

    def initialize_entry_boxes(self):
        self.entry = tk.Entry(
            fg = 'black',
            bg = 'white',
            width = 25,
            borderwidth = 2,
        )
        self.entry.insert(0, '3*pi*exp(-5*sin(2*pi*x))') # Initialize contents of textbox
        self.entry.pack()

    def initialize_buttons(self):
        self.plot_button = tk.Button(
            text = 'Plot function',
            width = 10,
            height = 2,
            bg = 'grey',
            fg = 'white',
            command = self.plot_button_handler,
        )
        self.plot_button.pack()

    def plot_button_handler(self):
        self.canvas.flush_events() # Clear the canvas every time you try to plot a function
        self.plot_object.cla() # Clear the axis every time you try to plot a function

        x = var('x')  # the possible variable names must be known beforehand...        
        user_input = self.entry.get()   

        expr_temp = sympify(user_input) # Convert user input into mathematical format understood by the python interpreter

        # Evaluate the expression for a range of values
        expr = []
        for i in np.arange(-1.0, 1.0, 0.05):
            expr.append(expr_temp.evalf(subs = {x:i}))  

        # Plot
        x = np.arange(-1, 1, 0.05)
        self.plot_object.plot(x, expr, color = 'red')
        self.plot_object.grid()
        self.plot_object.set_xlabel('Inpur variable - x', fontsize = 16)
        expr_in_tex = latex(expr_temp, mode = 'inline')
        self.plot_object.set_ylabel(expr_in_tex, fontsize = 16)
        self.canvas.draw()                   

    def destroy_window(self):
        self.window.destroy() # Close the window

def main():
    # Create a Plotting object
    window = tk.Tk() # Create Tkinter window
    window_manager = WindowManagement(window, 'Real-time Plotting')
    window.mainloop()

if __name__ == "__main__":
    main()
