import React, { useState } from 'react';
import { MousePointer2, Eclipse, MoveRight, Activity, Link, Minus, Crosshair, Scan, Globe, Layers, Settings, Combine, Scissors, ChevronDown, ChevronRight, Calculator, Pentagon, Zap } from 'lucide-react';
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
                <div className="flex flex-col gap-1 p-1 mt-1 border-l border-slate-700 ml-2 pl-2">
                    {children}
                </div>
            </div>
        </div>
    );
};

const ToolBtn = ({ mode, setMode, target, icon, label }: { mode: string, setMode: (m: string) => void, target: string, icon: React.ReactNode, label: string }) => (
    <button 
        onClick={() => setMode(target)}
        className={`p-1.5 w-full rounded transition flex flex-col items-center ${mode === target ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
        title={label}
    >
        {icon}
        <span className="text-[9px] mt-0.5 scale-90">{label}</span>
    </button>
);

// SVGs
const SquareIcon = () => <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor"><rect x="3" y="3" width="18" height="18" rx="2" /></svg>;
const CircleIcon = () => <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor"><circle cx="12" cy="12" r="10" /></svg>;
const RampIcon = () => <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor"><path d="M22 20L2 20L22 4V20Z" /></svg>;
const ParticleIcon = () => <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor"><circle cx="12" cy="12" r="4" /></svg>;

export const Toolbar: React.FC<ToolbarProps> = ({ 
    dragMode, setDragMode, setPolyBuilder, setArcBuilder, setRampBuilder, setCombineSelection, setShowEquationBuilder 
}) => {
    return (
        <aside className="w-20 bg-slate-900 border-r border-slate-800 flex flex-col py-2 overflow-y-auto no-scrollbar select-none z-10 items-center">
              <div className="flex flex-col items-center gap-1 mb-2 w-full px-1">
                 <button 
                      onClick={() => setDragMode('select')}
                      className={`p-2 w-full rounded-lg transition flex flex-col items-center ${dragMode === 'select' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                      title="选择 (Select)"
                  >
                      <MousePointer2 size={20} />
                      <span className="text-[9px] mt-1">选择</span>
                  </button>
              </div>

              {/* Group: Shapes */}
              <CollapsibleGroup icon={<SquareIcon />} label="形状" defaultOpen={true}>
                  <ToolBtn mode={dragMode} setMode={setDragMode} target="add_circle" icon={<CircleIcon />} label="圆形" />
                  <ToolBtn mode={dragMode} setMode={setDragMode} target="add_box" icon={<SquareIcon />} label="矩形" />
                  <ToolBtn mode={dragMode} setMode={setDragMode} target="add_particle" icon={<ParticleIcon />} label="质点" />
                  <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setPolyBuilder([]); }} target="add_poly" icon={<Pentagon size={18} />} label="多边形" />
                  <button onClick={() => setShowEquationBuilder(true)} className="p-1.5 w-full rounded transition flex flex-col items-center text-slate-400 hover:bg-slate-800" title="方程生成">
                      <Calculator size={18} />
                      <span className="text-[9px] mt-0.5 scale-90">方程</span>
                  </button>
              </CollapsibleGroup>

              {/* Group: Environment */}
              <CollapsibleGroup icon={<RampIcon />} label="环境">
                  <button 
                      onClick={() => { setDragMode('add_arc'); setArcBuilder(null); }}
                      className={`p-1.5 w-full rounded transition flex flex-col items-center ${dragMode === 'add_arc' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                      title="弧形轨道"
                  >
                      <Eclipse size={18} />
                      <span className="text-[9px] mt-0.5 scale-90">轨道</span>
                  </button>
                   <button 
                      onClick={() => { setDragMode('add_ramp'); setRampBuilder(null); }}
                      className={`p-1.5 w-full rounded transition flex flex-col items-center ${dragMode === 'add_ramp' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                      title="斜面"
                  >
                      <RampIcon />
                      <span className="text-[9px] mt-0.5 scale-90">斜面</span>
                  </button>
                  <button 
                      onClick={() => { setDragMode('add_conveyor'); setRampBuilder(null); }}
                      className={`p-1.5 w-full rounded transition flex flex-col items-center ${dragMode === 'add_conveyor' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                      title="传送带"
                  >
                      <MoveRight size={18} />
                      <span className="text-[9px] mt-0.5 scale-90">传送带</span>
                  </button>
              </CollapsibleGroup>

              {/* Group: Linkage */}
              <CollapsibleGroup icon={<Link size={16} />} label="连接">
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_spring" icon={<Activity size={18} />} label="弹簧" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_rod" icon={<Link size={18} />} label="刚性杆" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_rope" icon={<Minus size={18} className="rotate-45" />} label="轻绳" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_pin" icon={<Crosshair size={18} />} label="销钉" />
              </CollapsibleGroup>

              {/* Group: Fields */}
              <CollapsibleGroup icon={<Zap size={16} />} label="场">
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_field_box" icon={<Scan size={18} />} label="矩形场" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_field_circle" icon={<Globe size={18} />} label="圆形场" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="add_field_poly" icon={<Layers size={18} />} label="多边形" />
              </CollapsibleGroup>

              {/* Group: Tools */}
              <CollapsibleGroup icon={<Settings size={16} />} label="工具">
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="tool_velocity" icon={<MoveRight size={18} className="-rotate-45" />} label="速度" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="tool_force" icon={<Zap size={18} />} label="恒力" />
                   <ToolBtn mode={dragMode} setMode={(m) => { setDragMode(m); setCombineSelection([]); }} target="tool_combine" icon={<Combine size={18} />} label="合并" />
                   <ToolBtn mode={dragMode} setMode={setDragMode} target="tool_cut" icon={<Scissors size={18} />} label="切割" />
              </CollapsibleGroup>
          </aside>
    );
};