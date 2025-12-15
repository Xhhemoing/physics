
import React, { RefObject } from 'react';
import { Activity, Play, Pause, RotateCcw, Timer, Settings, Camera, Download, Upload, Trash2 } from 'lucide-react';
import { SimulationState } from '../types';
import { CanvasRef } from './SimulationCanvas';

interface HeaderProps {
    state: SimulationState;
    onTogglePause: () => void;
    onReset: () => void;
    onClearAll: () => void;
    onPreset: (type: 'smooth' | 'elastic') => void;
    onSaveScene: () => void;
    onImportScene: (e: React.ChangeEvent<HTMLInputElement>) => void;
    canvasRef: RefObject<CanvasRef | null>;
    showSnapshots: boolean;
    setShowSnapshots: (v: boolean) => void;
    showSettings: boolean;
    setShowSettings: (v: boolean) => void;
    onChangeName: (name: string) => void;
}

export const Header: React.FC<HeaderProps> = ({ 
    state, onTogglePause, onReset, onClearAll, onPreset, onSaveScene, onImportScene, 
    canvasRef, showSnapshots, setShowSnapshots, showSettings, setShowSettings, onChangeName 
}) => {
    return (
      <header className="h-12 bg-slate-900 border-b border-slate-800 flex items-center justify-between px-4 z-20">
        <div className="flex items-center space-x-2">
            <Activity className="text-blue-500" />
            <h1 className="font-bold text-lg tracking-tight bg-gradient-to-r from-blue-400 to-purple-400 bg-clip-text text-transparent hidden md:block">PhysLab Pro</h1>
            <input 
                type="text" 
                value={state.canvasName}
                onChange={(e) => onChangeName(e.target.value)}
                className="bg-slate-800 border border-slate-700 rounded px-2 py-0.5 text-xs text-slate-300 w-32 focus:w-48 transition-all outline-none focus:border-blue-500"
                placeholder="Project Name"
            />
        </div>
        
        <div className="flex items-center space-x-4">
             <div className="flex items-center bg-slate-800 rounded-lg p-1 space-x-1">
                 <button 
                    onClick={onTogglePause}
                    className={`p-1.5 rounded-md transition ${state.paused ? 'bg-green-600 hover:bg-green-500 text-white' : 'hover:bg-slate-700 text-slate-300'}`}
                    title={state.paused ? "Play" : "Pause"}
                 >
                     {state.paused ? <Play size={18} fill="currentColor" /> : <Pause size={18} fill="currentColor" />}
                 </button>
                 <button 
                    onClick={onReset}
                    className="p-1.5 rounded-md hover:bg-slate-700 text-slate-300 transition"
                    title="Reset Simulation"
                 >
                     <RotateCcw size={18} />
                 </button>
             </div>
             
             <div className="h-6 w-px bg-slate-700 mx-2" />
             
             <div className="flex items-center gap-2">
                 <button onClick={() => setShowSnapshots(!showSnapshots)} className={`p-1.5 rounded transition ${showSnapshots ? 'bg-amber-600 text-white' : 'text-slate-400 hover:text-amber-400'}`} title="Snapshots">
                    <Timer size={18} />
                 </button>
                 <button onClick={() => setShowSettings(!showSettings)} className={`p-1.5 rounded transition ${showSettings ? 'bg-purple-600 text-white' : 'text-slate-400 hover:text-purple-400'}`} title="Global Settings">
                    <Settings size={18} />
                 </button>
             </div>

             <div className="h-6 w-px bg-slate-700 mx-2 hidden sm:block" />

             <div className="flex space-x-2 text-xs hidden lg:flex">
                 <button onClick={() => onPreset('smooth')} className="px-3 py-1.5 bg-slate-800 hover:bg-slate-700 rounded transition" title="Set friction to 0">一键光滑</button>
                 <button onClick={() => onPreset('elastic')} className="px-3 py-1.5 bg-slate-800 hover:bg-slate-700 rounded transition">完全弹性</button>
                 <button onClick={onClearAll} className="px-3 py-1.5 bg-red-900/30 hover:bg-red-900/50 text-red-300 rounded transition">清空</button>
             </div>
        </div>

        <div className="flex items-center space-x-3">
             <button onClick={() => canvasRef.current?.exportImage()} className="p-2 hover:bg-slate-800 rounded text-slate-400 hover:text-white" title="Export Image">
                 <Camera size={18} />
             </button>
             <button onClick={onSaveScene} className="p-2 hover:bg-slate-800 rounded text-slate-400 hover:text-white" title="Save Scene">
                 <Download size={18} />
             </button>
             <label className="p-2 hover:bg-slate-800 rounded text-slate-400 hover:text-white cursor-pointer" title="Load Scene">
                 <Upload size={18} />
                 <input type="file" onChange={onImportScene} className="hidden" accept=".json" />
             </label>
        </div>
      </header>
    );
};
