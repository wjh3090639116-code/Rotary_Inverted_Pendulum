"""滚动容器组件：将滚轮操作限定在对应区域，保持主窗口布局稳定。"""
import tkinter as tk
from tkinter import ttk
class ScrollPanel(ttk.Frame):
    def __init__(self,parent,**kwargs):
        super().__init__(parent,**kwargs)
        self.canvas=tk.Canvas(self,width=310,height=1,highlightthickness=0)
        self.scroll=ttk.Scrollbar(self,orient='vertical',command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scroll.set)
        self.canvas.pack(side='left',fill='both',expand=True);self.scroll.pack(side='right',fill='y')
        self.body=ttk.Frame(self.canvas,padding=(2,0,2,4))
        self.item=self.canvas.create_window((0,0),window=self.body,anchor='nw')
        self.body.bind('<Configure>',lambda e:self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.canvas.bind('<Configure>',lambda e:self.canvas.itemconfigure(self.item,width=e.width))
        self.bind('<Map>',lambda e:self._bind_wheel(self.body))
        self.canvas.bind('<MouseWheel>',self._wheel)
    def _wheel(self,e):
        if self.body.winfo_reqheight()>self.canvas.winfo_height():self.canvas.yview_scroll(-int(e.delta/120),'units')
        return 'break'
    def _bind_wheel(self,w):
        w.bind('<MouseWheel>',self._wheel)
        for child in w.winfo_children():self._bind_wheel(child)
