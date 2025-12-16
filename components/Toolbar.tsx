
import React, { useState } from 'react';
import { MousePointer2, Circle, Box, MoveRight, Activity, Link, Minus, Crosshair, Scan, Globe, Layers, Settings, Combine, Scissors, ChevronDown, ChevronRight, Calculator, Pentagon, Zap, Triangle } from 'lucide-react';
import { Vector2 } from '../types';

interface ToolbarProps {
    dragMode: string;
    setDragMode: (mode: string) => void;
    setPolyBuilder: (pts: Vector2[]) => void;
    setArcBuilder: (v: any) => void;
    setRampBuilder: (v: any) => void;
    setCombineSelection: (ids: string[]) => void;
    setShowEquationBuilder: (show: boolean) => void;
}

const CollapsibleGroup = ({ icon, label, children, defaultOpen = false }: { icon: React.ReactNode, label: string, children?: React.ReactNode, defaultOpen?: boolean }) => {
    const [isOpen, setIsOpen] = useState(defaultOpen);
    return (
        <div className="w-full px-1 mb-1">
            <button 
                onClick={() => setIsOpen(!isOpen)}
                className={`flex items-center justify-between w-full p-2 rounded text-[10px] font-bold uppercase tracking-wider transition-colors ${isOpen ? 'bg-slate-800 text-slate-200' : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800/50'}`}
            >
                <div className="flex items-center gap-2">
                    {icon}
                    <span>{label}</span>
                </div>
                {isOpen ? <ChevronDown size={12} /> : <ChevronRight size={12} />}
            </button>
            <div className={`overflow-hidden transition-all duration-300 ${isOpen ? 'max-h-96 opacity-100' : 'max-h-0 opacity-0'}`}>
                <div className="grid grid-cols-2 gap-1 p-1 mt-1 border-l border-slate-700 ml-2 pl-2">
                    {children}
                </div>
            </div>
        </div>
    );
};

const ToolBtn = ({ mode, setMode, target, icon, label }: { mode: string, setMode: (m: string) => void, target: string, icon: React.ReactNode, label: string }) => (
    <button 
        onClick={() => setMode(target)}
        className={`p-1.5 w-full rounded transition flex flex-col items-center justify-center aspect-square ${mode === target ? 'bg-blue-600 text-white shadow-lg shadow-blue-900/50' : 'text-slate-400 hover:bg-slate-800'}`}
        title={label}
    >
        {icon}
        <span className="text-[9px] mt-1 scale-90 truncate w-full text-center">{label}</span>
    </button>
);

export const Toolbar: React.FC<ToolbarProps> = ({ 
    dragMode, setDragMode, setPolyBuilder, setArcBuilder, setRampBuilder, setCombineSelection, setShowEquationBuilder 
}) => {
    return (
        <aside className="w-20 bg-slate-900 border-r border-slate-800 flex flex-col py-2 overflow-y-auto no-scrollbar select-none z-10 items-center">
              <div className="flex flex-col items-center gap-1 mb-2 w-full px-2">
                 <button 
                      onClick={() => setDragMode('select')}
                      className={`p-2 w-full rounded-lg transition flex flex-col items-center ${dragMode === 'select' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                      title="Select / Move"
                  >
                      <MousePointer2 size={24} />
                      <span className="text-[10px] mt-1 font-bold">选择</span>
                  </button>
              </div>

              <CollapsibleGroup icon={<Box size={14} />} label="创造" defaultOpen={true}>
                  <ToolBtn mode={dragMode} setMode={setDragMode} target="add_circle" icon={<Circle size={18} />} label="圆形" />
                  <ToolBtn mode={dragMode} setMode={setDragMode} target="add_box" icon={<Box size={18} />} label="矩形" />
                  <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setPolyBuilder([]); }} target="add_poly" icon={<Pentagon size={18} />} label="多边形" />
                  <ToolBtn mode={dragMode} setMode={setDragMode} target="add_particle" icon={<Circle size={10} />} label="质点" />
                  <button onClick={() => setShowEquationBuilder(true)} className="p-1.5 w-full rounded transition flex flex-col items-center aspect-square justify-center text-slate-400 hover:bg-slate-800" title="Equation Body">
                      <Calculator size={18} />
                      <span className="text-[9px] mt-1 scale-90">方程</span>
                  </button>
              </CollapsibleGroup>

              <CollapsibleGroup icon={<Triangle size={14} />} label="地形">
                  <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setRampBuilder(null); }} target="add_ramp" icon={<Triangle size={18} />} label="斜面" />
                  <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setArcBuilder(null); }} target="add_arc" icon={<Globe size={18} />} label="轨道" />
                  <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setRampBuilder(null); }} target="add_conveyor" icon={<MoveRight size={18} />} label="传送带" />
              </CollapsibleGroup>

              <CollapsibleGroup icon={<Link size={14} />} label="连接">
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_spring" icon={<Activity size={18} />} label="弹簧" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_rod" icon={<Link size={18} />} label="杆" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_rope" icon={<Minus size={18} className="rotate-45" />} label="绳" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_pin" icon={<Crosshair size={18} />} label="销轴" />
              </CollapsibleGroup>

              <CollapsibleGroup icon={<Zap size={14} />} label="场">
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_field_box" icon={<Scan size={18} />} label="方形场" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_field_circle" icon={<Circle size={18} className="border-dashed" />} label="圆形场" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_field_poly" icon={<Layers size={18} />} label="多边形场" />
              </CollapsibleGroup>

              <CollapsibleGroup icon={<Settings size={14} />} label="工具">
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="tool_velocity" icon={<MoveRight size={18} className="-rotate-45" />} label="速度" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="tool_force" icon={<Zap size={18} />} label="力" />
                   <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setCombineSelection([]); }} target="tool_combine" icon={<Combine size={18} />} label="组合" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="tool_cut" icon={<Scissors size={18} />} label="切割" />
              </CollapsibleGroup>
          </aside>
    );
};
