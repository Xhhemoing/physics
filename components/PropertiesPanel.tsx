
import React from 'react';
import { PhysicsBody, BodyType, PhysicsField, FieldType, Vector2, FieldShape } from '../types';

interface Props {
  body: PhysicsBody | null;
  field: PhysicsField | null;
  onUpdateBody: (id: string, updates: Partial<PhysicsBody>) => void;
  onUpdateField: (id: string, updates: Partial<PhysicsField>) => void;
  onDeleteBody: (id: string) => void;
  onDeleteField: (id: string) => void;
}

const PropertiesPanel: React.FC<Props> = ({ body, field, onUpdateBody, onUpdateField, onDeleteBody, onDeleteField }) => {
  
  if (field) {
      return (
          <div className="p-4 space-y-5">
              <div className="flex justify-between items-center border-b border-slate-700 pb-3">
                <div>
                    <span className="text-xs text-slate-500 uppercase tracking-wider block">当前选择 (Field)</span>
                    <div className="flex items-center gap-2">
                        <h2 className="font-bold text-lg text-emerald-400">{field.shape} Field</h2>
                    </div>
                </div>
                <button 
                    onClick={() => onDeleteField(field.id)}
                    className="text-red-400 hover:text-red-300 text-xs px-3 py-1.5 rounded bg-red-900/20 hover:bg-red-900/40 transition"
                >
                    删除
                </button>
              </div>

              <div className="space-y-4">
                  <div>
                      <label className="block text-[10px] text-slate-400 mb-1 uppercase">类型 Type</label>
                      <select 
                          value={field.type}
                          onChange={(e) => onUpdateField(field.id, { type: e.target.value as FieldType })}
                          className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm text-white outline-none"
                      >
                          <option value={FieldType.UNIFORM_ELECTRIC}>电场 (Uniform Electric)</option>
                          <option value={FieldType.UNIFORM_MAGNETIC}>磁场 (Uniform Magnetic)</option>
                          <option value={FieldType.AREA_GRAVITY}>重力场 (Gravity Zone)</option>
                          <option value={FieldType.CUSTOM}>自定义方程 (Custom Function)</option>
                      </select>
                  </div>

                  {field.type === FieldType.CUSTOM ? (
                      <div className="space-y-3 p-2 bg-slate-800/50 rounded border border-slate-700">
                          <p className="text-[10px] text-slate-400">输入 JavaScript 表达式，可用变量: x, y, t, Math.*</p>
                          <div>
                              <label className="block text-[10px] text-pink-400 mb-1 font-mono">Ex(x,y) =</label>
                              <textarea
                                  value={field.equations?.ex || "0"}
                                  onChange={(e) => onUpdateField(field.id, { equations: { ex: e.target.value, ey: field.equations?.ey || "0" } })}
                                  className="w-full bg-slate-900 border border-slate-600 rounded px-2 py-1 text-xs font-mono h-16 outline-none resize-none"
                              />
                          </div>
                          <div>
                              <label className="block text-[10px] text-pink-400 mb-1 font-mono">Ey(x,y) =</label>
                              <textarea
                                  value={field.equations?.ey || "0"}
                                  onChange={(e) => onUpdateField(field.id, { equations: { ey: e.target.value, ex: field.equations?.ex || "0" } })}
                                  className="w-full bg-slate-900 border border-slate-600 rounded px-2 py-1 text-xs font-mono h-16 outline-none resize-none"
                              />
                          </div>
                      </div>
                  ) : field.type === FieldType.UNIFORM_MAGNETIC ? (
                      <div>
                          <label className="block text-[10px] text-slate-400 mb-1 uppercase">强度 Strength (T)</label>
                          <input 
                              type="number" step="0.1"
                              value={field.strength as number} 
                              onChange={(e) => onUpdateField(field.id, { strength: parseFloat(e.target.value) })}
                              className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                          />
                      </div>
                  ) : (
                      <div className="grid grid-cols-2 gap-2">
                           <div>
                              <label className="block text-[10px] text-slate-400 mb-1 uppercase">X 强度</label>
                              <input 
                                  type="number" step="1"
                                  value={(field.strength as Vector2).x} 
                                  onChange={(e) => onUpdateField(field.id, { strength: { ...(field.strength as Vector2), x: parseFloat(e.target.value) } })}
                                  className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                              />
                           </div>
                           <div>
                              <label className="block text-[10px] text-slate-400 mb-1 uppercase">Y 强度</label>
                              <input 
                                  type="number" step="1"
                                  value={(field.strength as Vector2).y} 
                                  onChange={(e) => onUpdateField(field.id, { strength: { ...(field.strength as Vector2), y: parseFloat(e.target.value) } })}
                                  className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                              />
                           </div>
                      </div>
                  )}
                  
                  {field.shape === FieldShape.CIRCLE && (
                      <div>
                          <label className="block text-[10px] text-slate-400 mb-1 uppercase">半径 Radius</label>
                          <input 
                              type="number" 
                              value={field.radius} 
                              onChange={(e) => onUpdateField(field.id, { radius: parseFloat(e.target.value) })}
                              className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                          />
                      </div>
                  )}

                   {field.shape === FieldShape.BOX && (
                      <div className="grid grid-cols-2 gap-2">
                           <div>
                              <label className="block text-[10px] text-slate-400 mb-1 uppercase">宽度 Width</label>
                              <input 
                                  type="number" 
                                  value={field.size.x} 
                                  onChange={(e) => onUpdateField(field.id, { size: { ...field.size, x: parseFloat(e.target.value) } })}
                                  className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                              />
                           </div>
                           <div>
                              <label className="block text-[10px] text-slate-400 mb-1 uppercase">高度 Height</label>
                              <input 
                                  type="number" 
                                  value={field.size.y} 
                                  onChange={(e) => onUpdateField(field.id, { size: { ...field.size, y: parseFloat(e.target.value) } })}
                                  className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                              />
                           </div>
                      </div>
                  )}
              </div>
          </div>
      );
  }

  if (!body) {
    return (
        <div className="p-4 text-slate-500 text-center text-sm flex flex-col items-center justify-center h-full opacity-50">
            <div className="mb-2 text-4xl">∅</div>
            <p>未选择对象 (No Selection)</p>
        </div>
    );
  }

  const handleChange = (key: keyof PhysicsBody, value: string | number) => {
    let val = Number(value);
    if (key === 'mass') {
        onUpdateBody(body.id, { mass: val, inverseMass: val === 0 ? 0 : 1/val });
    } else {
        onUpdateBody(body.id, { [key]: val });
    }
  };
  
  return (
    <div className="p-4 space-y-5">
      <div className="flex justify-between items-center border-b border-slate-700 pb-3">
        <div>
            <span className="text-xs text-slate-500 uppercase tracking-wider block">当前选择</span>
            <div className="flex items-center gap-2">
                <h2 className="font-bold text-lg text-amber-400">{body.isParticle ? 'PARTICLE' : body.type}</h2>
                {body.isParticle && <span className="text-[10px] bg-yellow-900 text-yellow-200 px-1 rounded border border-yellow-700">质点</span>}
            </div>
        </div>
        <button 
            onClick={() => onDeleteBody(body.id)}
            className="text-red-400 hover:text-red-300 text-xs px-3 py-1.5 rounded bg-red-900/20 hover:bg-red-900/40 transition"
        >
            删除 (Delete)
        </button>
      </div>

      <div className="space-y-4">
        {/* Research / Visual Tools */}
        <div className="space-y-3">
            <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-emerald-500 pl-2">可视分析 (Visual Analysis)</h3>
            
             <div className="grid grid-cols-2 gap-2">
                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        id="traj"
                        checked={body.showTrajectory || false}
                        onChange={(e) => onUpdateBody(body.id, { showTrajectory: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label htmlFor="traj" className="text-xs text-slate-400">轨迹 (Trail)</label>
                 </div>
                 
                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        id="showVel"
                        checked={body.showVelocity || false}
                        onChange={(e) => onUpdateBody(body.id, { showVelocity: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label htmlFor="showVel" className="text-xs text-slate-400">速度 (Vel)</label>
                 </div>

                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        id="showAcc"
                        checked={body.showAcceleration || false}
                        onChange={(e) => onUpdateBody(body.id, { showAcceleration: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label htmlFor="showAcc" className="text-xs text-slate-400">加速度 (Acc)</label>
                 </div>

                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        id="showForce"
                        checked={body.showForce || false}
                        onChange={(e) => onUpdateBody(body.id, { showForce: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label htmlFor="showForce" className="text-xs text-slate-400">受力 (Force)</label>
                 </div>
                 
                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        id="showCharge"
                        checked={body.showCharge || false}
                        onChange={(e) => onUpdateBody(body.id, { showCharge: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label htmlFor="showCharge" className="text-xs text-slate-400">电荷 (Charge)</label>
                 </div>
             </div>
        </div>

        {/* Physical Props */}
        <div className="space-y-3">
            <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-blue-500 pl-2">物理属性 (Physics)</h3>
            
            <div className="grid grid-cols-2 gap-3">
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">质量 Mass (kg)</label>
                    <input 
                        type="number" min="0"
                        value={body.mass} 
                        onChange={(e) => handleChange('mass', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                    />
                </div>
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">电荷 Charge (C)</label>
                    <input 
                        type="number" 
                        value={body.charge} 
                        onChange={(e) => handleChange('charge', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                    />
                </div>
            </div>

            <div className="grid grid-cols-2 gap-3">
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">弹性 Restitution</label>
                    <input 
                        type="number" step="0.1" min="0" max="1"
                        value={body.restitution} 
                        onChange={(e) => handleChange('restitution', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                    />
                </div>
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">摩擦 Friction</label>
                    <input 
                        type="number" step="0.1" min="0" max="1"
                        value={body.friction} 
                        onChange={(e) => handleChange('friction', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                    />
                </div>
            </div>

             {body.surfaceSpeed !== undefined && (
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">传送带速度 Surface Speed</label>
                    <input 
                        type="number" 
                        value={body.surfaceSpeed} 
                        onChange={(e) => handleChange('surfaceSpeed', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-purple-500 outline-none transition"
                    />
                </div>
            )}
        </div>

        {/* Geometry */}
        <div className="space-y-3">
            <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-purple-500 pl-2">几何形状 (Geometry)</h3>
            
            {!body.isParticle && (body.type === BodyType.CIRCLE || body.type === BodyType.ARC) && (
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">半径 Radius (m)</label>
                    <input 
                        type="number" 
                        value={body.radius} 
                        onChange={(e) => handleChange('radius', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                    />
                </div>
            )}
            
            {body.type === BodyType.BOX && (
                <div className="grid grid-cols-2 gap-3">
                    <div>
                        <label className="block text-[10px] text-slate-400 mb-1 uppercase">宽度 Width</label>
                        <input 
                            type="number" 
                            value={body.width} 
                            onChange={(e) => handleChange('width', e.target.value)}
                            className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                        />
                    </div>
                    <div>
                        <label className="block text-[10px] text-slate-400 mb-1 uppercase">高度 Height</label>
                        <input 
                            type="number" 
                            value={body.height} 
                            onChange={(e) => handleChange('height', e.target.value)}
                            className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                        />
                    </div>
                </div>
            )}

            <div>
                 <label className="block text-[10px] text-slate-400 mb-1 uppercase">位置 Position (x, y)</label>
                 <div className="flex gap-2 font-mono text-xs">
                     <div className="bg-slate-900/50 text-slate-400 border border-slate-700 rounded px-2 py-1.5 flex-1">
                         X: {body.position.x.toFixed(2)}
                     </div>
                     <div className="bg-slate-900/50 text-slate-400 border border-slate-700 rounded px-2 py-1.5 flex-1">
                         Y: {(-body.position.y).toFixed(2)}
                     </div>
                 </div>
            </div>

            <div>
                 <label className="block text-[10px] text-slate-400 mb-1 uppercase">角度 Angle (rad)</label>
                 <input 
                        type="number" step="0.1"
                        value={body.angle} 
                        onChange={(e) => handleChange('angle', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm focus:border-blue-500 outline-none transition"
                    />
            </div>
        </div>
      </div>
    </div>
  );
};

export default PropertiesPanel;
