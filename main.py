'''CLI demo and optional Tkinter interactive grid editor.'''
import argparse
from pathlib import Path
from navigation import Grid, Robot, parse_ascii, plan, render

DEFAULT_MAP = '''\
S...............
.#####...####...
.....#......#...
..##.#..###.#...
.....#......#...
.#####.####.#...
.............G..
'''

def launch_gui():
    import tkinter as tk
    root = tk.Tk(); root.title('Autonomous Navigation Simulator')
    grid = Grid(24, 16, set()); start=(1,1); goal=(22,14); robot=Robot(grid,start,goal)
    cell=30
    info=tk.StringVar(value='Left-click to add/remove obstacles. Run simulation or reset.')
    canvas=tk.Canvas(root,width=grid.width*cell,height=grid.height*cell,bg='#0f172a')
    canvas.pack(padx=12,pady=12)
    def draw():
        canvas.delete('all')
        route=set(robot.path)
        for y in range(grid.height):
            for x in range(grid.width):
                p=(x,y)
                color=('#ef4444' if p==robot.position else '#22c55e' if p==goal else '#64748b' if p in grid.blocked else '#38bdf8' if p in route else '#1e293b')
                canvas.create_rectangle(x*cell,y*cell,(x+1)*cell-1,(y+1)*cell-1,fill=color,outline='#0f172a')
        info.set(f'Position: {robot.position} | Path: {max(0,len(robot.path)-1)} steps remaining')
    def click(event):
        p=(event.x//cell,event.y//cell)
        if p==robot.position or p==goal or not grid.inside(p): return
        if p in grid.blocked: grid.blocked.remove(p)
        else: grid.blocked.add(p)
        robot.path=plan(grid,robot.position,goal);draw()
    def tick():
        if robot.step(): draw();root.after(130,tick)
        else: draw()
    def reset():
        nonlocal robot
        robot=Robot(grid,start,goal);draw()
    canvas.bind('<Button-1>',click)
    toolbar=tk.Frame(root);toolbar.pack()
    tk.Button(toolbar,text='Run / Replan',command=tick).pack(side='left',padx=6)
    tk.Button(toolbar,text='Reset Robot',command=reset).pack(side='left',padx=6)
    tk.Label(root,textvariable=info).pack(pady=8)
    draw();root.mainloop()

def main():
    parser=argparse.ArgumentParser(description='A* and Dijkstra grid route planning')
    parser.add_argument('--map',type=Path,help='ASCII map file')
    parser.add_argument('--algorithm',choices=['astar','dijkstra'],default='astar')
    parser.add_argument('--gui',action='store_true')
    args=parser.parse_args()
    if args.gui: launch_gui();return
    grid,start,goal=parse_ascii(args.map.read_text() if args.map else DEFAULT_MAP)
    path=plan(grid,start,goal,args.algorithm)
    print(render(grid,path,start,goal));print(f'Algorithm: {args.algorithm}; distance: {len(path)-1 if path else "unreachable"}')
if __name__=='__main__':main()
