
import React, { useEffect, useRef, useState } from 'react';
import * as d3 from 'd3';
import { PhysicsBody, SimulationState, ChartType } from '../types';
import { Vec2 } from '../services/vectorMath';

interface Props {
  selectedBodyId: string | null;
  state: SimulationState;
}

const HISTORY_LENGTH = 500;
// Extended data point structure
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
const dataHistory: DataPoint[] = [];

type AxisVariable = 't' | 'x' | 'y' | 'vx' | 'vy' | 'v' | 'ax' | 'ay' | 'a' | 'ke';
type AxisModifier = 'none' | 'sq' | 'abs' | 'sqrt';

const VarSelect = ({ val, onChange }: {val: string, onChange: (v: any) => void}) => (
    <select value={val} onChange={(e) => onChange(e.target.value)} className="bg-slate-800 text-[10px] w-12 rounded border border-slate-700">
        <option value="t">t</option>
        <option value="x">x</option>
        <option value="y">y</option>
        <option value="v">v</option>
        <option value="vx">vx</option>
        <option value="vy">vy</option>
        <option value="a">a</option>
        <option value="ax">ax</option>
        <option value="ay">ay</option>
        <option value="ke">Ek</option>
    </select>
);

const ModSelect = ({ val, onChange }: {val: string, onChange: (v: any) => void}) => (
    <select value={val} onChange={(e) => onChange(e.target.value)} className="bg-slate-800 text-[10px] w-12 rounded border border-slate-700">
        <option value="none">-</option>
        <option value="sq">^2</option>
        <option value="sqrt">√</option>
        <option value="abs">| |</option>
    </select>
);

const DataCharts: React.FC<Props> = ({ selectedBodyId, state }) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const svgRef = useRef<SVGSVGElement>(null);
  const lastTimeRef = useRef(0);
  
  // Modes
  const [mode, setMode] = useState<'simple' | 'advanced'>('simple');
  const [simpleChartType, setSimpleChartType] = useState<string>('v-t');
  
  // Advanced Config
  const [xAxisVar, setXAxisVar] = useState<AxisVariable>('t');
  const [xAxisMod, setXAxisMod] = useState<AxisModifier>('none');
  const [yAxisVar, setYAxisVar] = useState<AxisVariable>('v');
  const [yAxisMod, setYAxisMod] = useState<AxisModifier>('none');

  useEffect(() => {
    if (!selectedBodyId) {
       dataHistory.length = 0;
    }
  }, [selectedBodyId]);

  useEffect(() => {
    if (!selectedBodyId || state.paused) return;

    const body = state.bodies.find(b => b.id === selectedBodyId);
    if (!body) return;

    if (state.time - lastTimeRef.current > 0.05) {
        const v = Vec2.mag(body.velocity);
        const a = Vec2.mag(body.acceleration);
        const ke = 0.5 * body.mass * v * v;
        
        dataHistory.push({
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
        });
        if (dataHistory.length > HISTORY_LENGTH) dataHistory.shift();
        
        lastTimeRef.current = state.time;
        drawChart();
    }
  }, [state, selectedBodyId]);

  // Re-draw on config change
  useEffect(() => {
      drawChart();
  }, [mode, simpleChartType, xAxisVar, xAxisMod, yAxisVar, yAxisMod]);

  const getValue = (pt: DataPoint, v: AxisVariable, mod: AxisModifier): number => {
      let val = pt[v];
      if (mod === 'sq') return val * val;
      if (mod === 'abs') return Math.abs(val);
      if (mod === 'sqrt') return Math.sqrt(Math.abs(val));
      return val;
  };

  const drawChart = () => {
    if (!containerRef.current || !svgRef.current || dataHistory.length < 2) return;

    const width = containerRef.current.clientWidth;
    const height = 180;
    const margin = { top: 10, right: 10, bottom: 20, left: 40 };

    const svg = d3.select(svgRef.current);
    svg.selectAll("*").remove();

    let getX: (d: DataPoint) => number;
    let getY: (d: DataPoint) => number;
    let labelX = "";
    let labelY = "";

    if (mode === 'simple') {
        switch (simpleChartType) {
            case 'v-t': getX = d=>d.t; getY = d=>d.v; labelX="t (s)"; labelY="v (m/s)"; break;
            case 'a-t': getX = d=>d.t; getY = d=>d.a; labelX="t (s)"; labelY="a (m/s²)"; break;
            case 'x-y': getX = d=>d.x; getY = d=>d.y; labelX="x (m)"; labelY="y (m)"; break;
            case 'ke-t': getX = d=>d.t; getY = d=>d.ke; labelX="t (s)"; labelY="Ek (J)"; break;
            default: getX = d=>d.t; getY = d=>d.v;
        }
    } else {
        getX = (d) => getValue(d, xAxisVar, xAxisMod);
        getY = (d) => getValue(d, yAxisVar, yAxisMod);
        
        const formatLabel = (v: string, m: string) => {
            if (m === 'none') return v;
            if (m === 'sq') return `${v}²`;
            if (m === 'abs') return `|${v}|`;
            if (m === 'sqrt') return `√${v}`;
            return v;
        };
        labelX = formatLabel(xAxisVar, xAxisMod);
        labelY = formatLabel(yAxisVar, yAxisMod);
    }

    const xExtent = d3.extent(dataHistory, getX) as [number, number];
    const yExtent = d3.extent(dataHistory, getY) as [number, number];
    
    // Safety check for NaNs or Infinity
    if (isNaN(xExtent[0]) || isNaN(yExtent[0])) return;

    const xRange = (xExtent[1] - xExtent[0]) || 1;
    const yRange = (yExtent[1] - yExtent[0]) || 1;

    const xScale = d3.scaleLinear()
      .domain([xExtent[0], xExtent[1]])
      .range([margin.left, width - margin.right]);

    const yScale = d3.scaleLinear()
      .domain([yExtent[0] - yRange*0.1, yExtent[1] + yRange*0.1])
      .range([height - margin.bottom, margin.top]);

    const line = d3.line<DataPoint>()
      .x(d => xScale(getX(d)))
      .y(d => yScale(getY(d)))
      .curve(d3.curveMonotoneX);

    // Axes
    svg.append("g")
      .attr("transform", `translate(0,${height - margin.bottom})`)
      .call(d3.axisBottom(xScale).ticks(5).tickSizeOuter(0))
      .attr("color", "#475569");

    svg.append("g")
      .attr("transform", `translate(${margin.left},0)`)
      .call(d3.axisLeft(yScale).ticks(5).tickSizeOuter(0))
      .attr("color", "#475569");

    // Grid
    svg.append("g")
        .attr("class", "grid")
        .attr("opacity", 0.1)
        .call(d3.axisLeft(yScale).ticks(5).tickSize(-width).tickFormat(() => ""));

    // Path
    svg.append("path")
      .datum(dataHistory)
      .attr("fill", "none")
      .attr("stroke", "#3b82f6")
      .attr("stroke-width", 2)
      .attr("d", line);

    // Labels
    svg.append("text")
        .attr("x", width / 2)
        .attr("y", height - 2)
        .attr("fill", "#64748b")
        .style("font-size", "10px")
        .style("text-anchor", "middle")
        .text(labelX);
        
    svg.append("text")
        .attr("x", 10)
        .attr("y", 10)
        .attr("fill", "#64748b")
        .style("font-size", "10px")
        .text(labelY);
  };

  if (!selectedBodyId) {
    return (
        <div className="flex items-center justify-center h-48 text-slate-500 text-sm italic">
            选择一个对象以查看实时数据
        </div>
    );
  }

  return (
    <div ref={containerRef} className="w-full h-full flex flex-col">
      <div className="flex justify-between items-center mb-2">
          <div className="flex space-x-2 bg-slate-800 p-0.5 rounded">
              <button 
                onClick={() => setMode('simple')}
                className={`text-[10px] px-2 py-0.5 rounded ${mode === 'simple' ? 'bg-blue-600 text-white' : 'text-slate-400'}`}
              >
                  简单
              </button>
              <button 
                onClick={() => setMode('advanced')}
                className={`text-[10px] px-2 py-0.5 rounded ${mode === 'advanced' ? 'bg-blue-600 text-white' : 'text-slate-400'}`}
              >
                  高级
              </button>
          </div>

          {mode === 'simple' && (
              <select 
                value={simpleChartType} 
                onChange={(e) => setSimpleChartType(e.target.value)}
                className="bg-slate-800 text-xs text-white border border-slate-600 rounded px-1 py-0.5 outline-none"
              >
                  <option value="v-t">v-t (速度)</option>
                  <option value="a-t">a-t (加速度)</option>
                  <option value="x-y">y-x (轨迹)</option>
                  <option value="ke-t">Ek-t (动能)</option>
              </select>
          )}
      </div>

      {mode === 'advanced' && (
          <div className="grid grid-cols-2 gap-2 mb-2 p-2 bg-slate-800/30 rounded border border-slate-800">
              <div className="flex flex-col gap-1">
                  <span className="text-[10px] text-slate-500">Y 轴</span>
                  <div className="flex gap-1">
                     <VarSelect val={yAxisVar} onChange={setYAxisVar} />
                     <ModSelect val={yAxisMod} onChange={setYAxisMod} />
                  </div>
              </div>
              <div className="flex flex-col gap-1">
                  <span className="text-[10px] text-slate-500">X 轴</span>
                  <div className="flex gap-1">
                     <VarSelect val={xAxisVar} onChange={setXAxisVar} />
                     <ModSelect val={xAxisMod} onChange={setXAxisMod} />
                  </div>
              </div>
          </div>
      )}

      <svg ref={svgRef} width="100%" height="180" className="bg-slate-900/50 rounded border border-slate-700 flex-1"></svg>
    </div>
  );
};

export default DataCharts;