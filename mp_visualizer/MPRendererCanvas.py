from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from commonroad.visualization.mp_renderer import MPRenderer

class MPRendererCanvas(FigureCanvas):
    """Matplotlib canvas specifically for CommonRoad visualization"""
    
    def __init__(self, figsize=(12,10)):
        self.figsize = figsize
        self.fig = Figure(figsize=figsize, constrained_layout=True)
        super().__init__(self.fig)
        self.mp_renderer = MPRenderer(figsize=figsize, 
                                      #plot_limits=[-15, 55, -55, 5]
                                      )        # Get the figure from the renderer
        
        self.mp_renderer.f = self.fig
        self.mp_renderer.ax = self.fig.add_subplot(111)  # Create axis on our figure
        
        # Set reasonable plot limits
        self.mp_renderer.ax.set_xlim(-20, 60)
        self.mp_renderer.ax.set_ylim(-55, 5)
        
        
        
        
    def clear(self):
        """Clear the renderer and figure"""
        self.mp_renderer.clear()
        #self.mp_renderer = MPRenderer(figsize=self.figsize, 
                                      #plot_limits=[-20,40,-90,0]
        #                              ) 
        #self.mp_renderer.f = self.fig
        #self.mp_renderer.ax.autoscale(enable=True)
        #self.mp_renderer.ay.autoscale(enable=False)
        #self.fig = self.mp_renderer.f
        #self.fig.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
        # self.fig.clf() 
        
    def render(self):
        """Render the plot and refresh the canvas"""
        self.mp_renderer.render(show=False)
        self.draw()
