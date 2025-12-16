

import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { SimulationState } from '../types';
import { Vec2 } from '../services/vectorMath';
import { Maximize2, Minimize2, X } from 'lucide-react';

interface Props {
  selectedBodyId: string | null;
  state: SimulationState;
  onClose?: () => void;
  width?: number;
  height?: number;
}

const HISTORY_LENGTH = 300;

interface DataPoint {
    t: number;
    x: number;
    y: number;
    vx: number;
    vy: number;
    v: number;
    ax: number;
    ay: number;
    a: number;
    ke: number;
}

const DataCharts: React.FC<Props> = ({ selectedBodyId, state, onClose, width = 300, height = 200 }) => {
  const svgRef = useRef<SVGSVGElement>(null);
  const lastTimeRef = useRef(0);
  const historyRef = useRef<DataPoint[]>([]);

  // Local state for chart config
  const [chartType, setChartType] = useState<string>('v-t');
  
  useEffect(() => {
    // Clear history if body changes
    historyRef.current = [];
  }, [selectedBodyId]);

  useEffect(() => {
    if (!selectedBodyId || state.paused) return;

    const body = state.bodies.find(b => b.id === selectedBodyId);
    if (!body) return;

    // Throttle updates
    if (state.time - lastTimeRef.current > 0.05) {
        const v = Vec2.mag(body.velocity);
        const a = Vec2.mag(body.acceleration);
        const ke = 0.5 * body.mass * v * v;
        
        const pt = {
            t: state.time,
            x: body.position.x,
            y: -body.position.y,
            vx: body.velocity.x,
            vy: -body.velocity.y,
            v: v,
            ax: body.acceleration.x,
            ay: -body.acceleration.y,
            a: a,
            ke: ke
        };

        historyRef.current.push(pt);
        if (historyRef.current.length > HISTORY_LENGTH) historyRef.current.shift();
        
        lastTimeRef.current = state.time;
        drawChart();
    }
  }, [state, selectedBodyId]);

  useEffect(() => {
      drawChart();
  }, [chartType, width, height]);

  const drawChart = () => {
    if (!svgRef.current || historyRef.current.length < 2) return;
    
    const margin = { top: 10, right: 10, bottom: 20, left: 35 };
    const w = width; 
    const h = height - 40; // Subtract header height

    const svg = d3.select(svgRef.current);
    svg.selectAll("*").remove();

    let getX: (d: DataPoint) => number;
    let getY: (d: DataPoint) => number;
    
    switch (chartType) {
        case 'v-t': getX = d=>d.t; getY = d=>d.v; break;
        case 'a-t': getX = d=>d.t; getY = d=>d.a; break;
        case 'x-y': getX = d=>d.x; getY = d=>d.y; break;
        case 'ke-t': getX = d=>d.t; getY = d=>d.ke; break;
        default: getX = d=>d.t; getY = d=>d.v;
    }

    const xExtent = d3.extent(historyRef.current, getX) as [number, number];
    const yExtent = d3.extent(historyRef.current, getY) as [number, number];

    if (!Number.isFinite(xExtent[0])) return;

    const xScale = d3.scaleLinear().domain(xExtent).range([margin.left, w - margin.right]);
    const yScale = d3.scaleLinear().domain(yExtent).range([h - margin.bottom, margin.top]);

    const line = d3.line<DataPoint>()
      .x(d => xScale(getX(d)))
      .y(d => yScale(getY(d)));

    svg.append("g")
      .attr("transform", `translate(0,${h - margin.bottom})`)
      .call(d3.axisBottom(xScale).ticks(4).tickFormat(d => d.valueOf().toFixed(1)))
      .attr("color", "#64748b");

    svg.append("g")
      .attr("transform", `translate(${margin.left},0)`)
      .call(d3.axisLeft(yScale).ticks(4).tickFormat(d => d.valueOf().toFixed(1)))
      .attr("color", "#64748b");

    svg.append("path")
      .datum(historyRef.current)
      .attr("fill", "none")
      .attr("stroke", "#3b82f6")
      .attr("stroke-width", 2)
      .attr("d", line);

    // Current Value Overlay
    const last = historyRef.current[historyRef.current.length-1];
    if (last) {
        svg.append("text")
           .attr("x", w - 10)
           .attr("y", 20)
           .attr("text-anchor", "end")
           .attr("fill", "#fff")
           .style("font-size", "12px")
           .text(getY(last).toFixed(2));
    }
  };

  if (!selectedBodyId) return <div className="p-4 text-xs text-slate-500">选择对象以显示数据</div>;

  return (
    <div className="flex flex-col h-full w-full bg-slate-900/90 text-white rounded overflow-hidden select-none">
       {/* Header */}
       <div className="flex justify-between items-center p-1 bg-slate-800 border-b border-slate-700 cursor-move draggable-handle">
           <select 
                value={chartType} 
                onChange={(e) => setChartType(e.target.value)}
                className="bg-transparent text-xs outline-none cursor-pointer"
                onMouseDown={e => e.stopPropagation()}
            >
                <option value="v-t">速度 (v-t)</option>
                <option value="a-t">加速度 (a-t)</option>
                <option value="x-y">轨迹 (y-x)</option>
                <option value="ke-t">动能 (Ek-t)</option>
           </select>
           {onClose && (
               <button onMouseDown={e => e.stopPropagation()} onClick={onClose} className="text-slate-400 hover:text-white">
                   <X size={14} />
               </button>
           )}
       </div>
       <div className="flex-1 bg-slate-900/50">
           <svg ref={svgRef} width="100%" height="100%"></svg>
       </div>
    </div>
  );
};

export default DataCharts;
