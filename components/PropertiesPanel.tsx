
import React from 'react';
import { PhysicsBody, BodyType, PhysicsField, FieldType, Vector2, FieldShape, Constraint, ConstraintType } from '../types';

interface Props {
  body: PhysicsBody | null;
  field: PhysicsField | null;
  constraint: Constraint | null; 
  onUpdateBody: (id: string, updates: Partial<PhysicsBody>) => void;
  onUpdateField: (id: string, updates: Partial<PhysicsField>) => void;
  onUpdateConstraint: (id: string, updates: Partial<Constraint>) => void; 
  onDeleteBody: (id: string) => void;
  onDeleteField: (id: string) => void;
  onDeleteConstraint: (id: string) => void; 
}

const PropertiesPanel: React.FC<Props> = ({ body, field, constraint, onUpdateBody, onUpdateField, onUpdateConstraint, onDeleteBody, onDeleteField, onDeleteConstraint }) => {
  
  const insertText = (text: string, axis: 'ex' | 'ey') => {
      if (!field || field.type !== FieldType.CUSTOM) return;
      const current = field.equations?.[axis] || "";
      onUpdateField(field.id, { 
          equations: { 
              ...field.equations!, 
              [axis]: current + text 
          } 
      });
  };

  // --- Constraint Panel ---
  if (constraint) {
      return (
          <div className="p-4 space-y-5">
              <div className="flex justify-between items-center border-b border-slate-700 pb-3">
                <div>
                    <span className="text-xs text-slate-500 uppercase tracking-wider block">当前选择 (Constraint)</span>
                    <div className="flex items-center gap-2">
                        <h2 className="font-bold text-lg text-amber-400">
                            {constraint.type === ConstraintType.SPRING ? '弹簧 (Spring)' : 
                             constraint.type === ConstraintType.ROD ? '刚性杆 (Rod)' : 
                             constraint.type === ConstraintType.ROPE ? '轻绳 (Rope)' :
                             '销轴 (Pin Joint)'}
                        </h2>
                    </div>
                </div>
                <button 
                    onClick={() => onDeleteConstraint(constraint.id)}
                    className="text-red-400 hover:text-red-300 text-xs px-3 py-1.5 rounded bg-red-900/20 hover:bg-red-900/40 transition"
                >
                    删除
                </button>
              </div>

              <div className="space-y-4">
                  {constraint.type === ConstraintType.SPRING && (
                      <>
                        <div>
                            <label className="block text-[10px] text-slate-400 mb-1 uppercase">劲度系数 (k)</label>
                            <input 
                                type="number" step="0.1"
                                value={constraint.stiffness} 
                                onChange={(e) => onUpdateConstraint(constraint.id, { stiffness: parseFloat(e.target.value) })}
                                className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                            />
                        </div>
                        <div>
                            <label className="block text-[10px] text-slate-400 mb-1 uppercase">阻尼 Damping</label>
                            <input 
                                type="number" step="0.01"
                                value={constraint.damping} 
                                onChange={(e) => onUpdateConstraint(constraint.id, { damping: parseFloat(e.target.value) })}
                                className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                            />
                        </div>
                      </>
                  )}
                  
                  {(constraint.type === ConstraintType.SPRING || constraint.type === ConstraintType.ROD || constraint.type === ConstraintType.ROPE) && (
                      <div>
                          <label className="block text-[10px] text-slate-400 mb-1 uppercase">长度 Length</label>
                          <input 
                              type="number" step="1"
                              value={constraint.length} 
                              onChange={(e) => onUpdateConstraint(constraint.id, { length: parseFloat(e.target.value) })}
                              className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                          />
                      </div>
                  )}
              </div>
          </div>
      );
  }

  // --- Field Panel ---
  if (field) {
      return (
          <div className="p-4 space-y-5">
              <div className="flex justify-between items-center border-b border-slate-700 pb-3">
                <div>
                    <span className="text-xs text-slate-500 uppercase tracking-wider block">当前选择 (Field)</span>
                    <div className="flex items-center gap-2">
                        <h2 className="font-bold text-lg text-emerald-400">{field.shape} 场</h2>
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
                          <option value={FieldType.CUSTOM}>自定义方程 (Function)</option>
                      </select>
                  </div>

                  {field.type === FieldType.CUSTOM ? (
                      <div className="space-y-3 p-2 bg-slate-800/50 rounded border border-slate-700">
                          <div>
                              <label className="block text-[10px] text-slate-400 mb-1 uppercase">方程模式 Mode</label>
                              <div className="flex gap-2">
                                  <button 
                                    onClick={() => onUpdateField(field.id, { customType: 'ELECTRIC' })}
                                    className={`flex-1 text-xs py-1 rounded ${!field.customType || field.customType === 'ELECTRIC' ? 'bg-pink-600 text-white' : 'bg-slate-700 text-slate-400'}`}
                                  >
                                    电场 (Force)
                                  </button>
                                  <button 
                                    onClick={() => onUpdateField(field.id, { customType: 'MAGNETIC' })}
                                    className={`flex-1 text-xs py-1 rounded ${field.customType === 'MAGNETIC' ? 'bg-blue-600 text-white' : 'bg-slate-700 text-slate-400'}`}
                                  >
                                    磁场 (Velocity)
                                  </button>
                              </div>
                          </div>
                          
                          <div className="space-y-2">
                             <div className="flex flex-wrap gap-1">
                                {['sin(t)', 'cos(t)', 'x', 'y', 't', 'sqrt()', 'abs()'].map(fn => (
                                    <button 
                                        key={fn}
                                        onClick={() => insertText(fn, 'ex')} 
                                        className="bg-slate-700 hover:bg-slate-600 text-[10px] text-white rounded px-1.5 py-0.5"
                                    >
                                        Ex:{fn}
                                    </button>
                                ))}
                             </div>
                          </div>

                          <div>
                              <label className="block text-[10px] text-pink-400 mb-1 font-mono">
                                  {field.customType === 'MAGNETIC' ? '磁场强度 B =' : 'X 分量 Ex ='}
                              </label>
                              <textarea
                                  value={field.equations?.ex || "0"}
                                  onChange={(e) => onUpdateField(field.id, { equations: { ex: e.target.value, ey: field.equations?.ey || "0" } })}
                                  className="w-full bg-slate-900 border border-slate-600 rounded px-2 py-1 text-xs font-mono h-16 outline-none resize-none"
                              />
                          </div>
                          
                          {(!field.customType || field.customType === 'ELECTRIC') && (
                              <div>
                                  <label className="block text-[10px] text-pink-400 mb-1 font-mono">Y 分量 Ey =</label>
                                  <textarea
                                      value={field.equations?.ey || "0"}
                                      onChange={(e) => onUpdateField(field.id, { equations: { ey: e.target.value, ex: field.equations?.ex || "0" } })}
                                      className="w-full bg-slate-900 border border-slate-600 rounded px-2 py-1 text-xs font-mono h-16 outline-none resize-none"
                                  />
                              </div>
                          )}
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

  // --- Body Panel ---
  if (!body) {
    return (
        <div className="p-4 text-slate-500 text-center text-sm flex flex-col items-center justify-center h-full opacity-50">
            <div className="mb-2 text-4xl">∅</div>
            <p>未选择对象</p>
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
                <h2 className="font-bold text-lg text-amber-400">{body.label}</h2>
                <span className="text-xs text-slate-500">({body.type})</span>
            </div>
        </div>
        <button 
            onClick={() => onDeleteBody(body.id)}
            className="text-red-400 hover:text-red-300 text-xs px-3 py-1.5 rounded bg-red-900/20 hover:bg-red-900/40 transition"
        >
            删除
        </button>
      </div>

      <div className="space-y-4">
        {/* Custom Image */}
        <div>
             <label className="block text-[10px] text-slate-400 mb-1 uppercase">自定义贴图 (URL)</label>
             <input 
                 type="text" 
                 value={body.image || ''}
                 onChange={(e) => onUpdateBody(body.id, { image: e.target.value })}
                 placeholder="https://..."
                 className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
             />
        </div>

        {/* Visual Analysis */}
        <div className="space-y-3">
            <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-emerald-500 pl-2">可视分析</h3>
            
             <div className="grid grid-cols-2 gap-2">
                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        checked={body.showTrajectory || false}
                        onChange={(e) => onUpdateBody(body.id, { showTrajectory: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label className="text-xs text-slate-400">轨迹</label>
                 </div>
                 
                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        checked={body.showVelocity || false}
                        onChange={(e) => onUpdateBody(body.id, { showVelocity: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label className="text-xs text-slate-400">速度</label>
                 </div>

                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        checked={body.showAcceleration || false}
                        onChange={(e) => onUpdateBody(body.id, { showAcceleration: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label className="text-xs text-slate-400">加速度</label>
                 </div>

                 <div className="flex items-center space-x-2">
                     <input 
                        type="checkbox" 
                        checked={body.showForce || false}
                        onChange={(e) => onUpdateBody(body.id, { showForce: e.target.checked })}
                        className="w-4 h-4 rounded border-slate-600 bg-slate-800"
                     />
                     <label className="text-xs text-slate-400">受力</label>
                 </div>
             </div>
        </div>
        
        {/* Custom Graph */}
        <div className="space-y-3">
             <div className="flex justify-between items-center">
                <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-pink-500 pl-2">自定义图像 (x,y)</h3>
                <input 
                    type="checkbox"
                    checked={body.customGraph?.show || false}
                    onChange={(e) => onUpdateBody(body.id, { customGraph: { ...body.customGraph!, show: e.target.checked } })}
                />
             </div>
             {body.customGraph?.show && (
                 <div className="bg-slate-800/50 p-2 rounded text-xs space-y-2">
                     <p className="text-[10px] text-slate-500">可用变量: x, y, vx, vy, v, ax, ay, a, m, ke</p>
                     <div>
                         <label className="block text-[10px] text-slate-400">X轴方程:</label>
                         <input 
                             type="text" 
                             value={body.customGraph.eqX}
                             onChange={(e) => onUpdateBody(body.id, { customGraph: { ...body.customGraph!, eqX: e.target.value } })}
                             className="w-full bg-slate-900 border border-slate-700 rounded px-1"
                         />
                     </div>
                     <div>
                         <label className="block text-[10px] text-slate-400">Y轴方程:</label>
                         <input 
                             type="text" 
                             value={body.customGraph.eqY}
                             onChange={(e) => onUpdateBody(body.id, { customGraph: { ...body.customGraph!, eqY: e.target.value } })}
                             className="w-full bg-slate-900 border border-slate-700 rounded px-1"
                         />
                     </div>
                 </div>
             )}
        </div>

        {/* Physics */}
        <div className="space-y-3">
            <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-blue-500 pl-2">物理属性</h3>
            
            {!body.isLine && (
                <div className="grid grid-cols-2 gap-3">
                    <div>
                        <label className="block text-[10px] text-slate-400 mb-1 uppercase">质量 (kg)</label>
                        <input 
                            type="number" min="0"
                            value={body.mass} 
                            onChange={(e) => handleChange('mass', e.target.value)}
                            className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                        />
                    </div>
                    <div>
                        <label className="block text-[10px] text-slate-400 mb-1 uppercase">电荷 (C)</label>
                        <input 
                            type="number" 
                            value={body.charge} 
                            onChange={(e) => handleChange('charge', e.target.value)}
                            className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                        />
                    </div>
                </div>
            )}

            <div className="grid grid-cols-2 gap-3">
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">弹性系数</label>
                    <input 
                        type="number" step="0.1" min="0" max="1"
                        value={body.restitution} 
                        onChange={(e) => handleChange('restitution', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                    />
                </div>
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">摩擦系数</label>
                    <input 
                        type="number" step="0.1" min="0" max="1"
                        value={body.friction} 
                        onChange={(e) => handleChange('friction', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                    />
                </div>
            </div>

             {body.surfaceSpeed !== undefined && (
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">传送带速度</label>
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
            <h3 className="text-xs font-semibold text-slate-300 border-l-2 border-purple-500 pl-2">几何属性</h3>
            
            {body.type === BodyType.LINE && (
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">长度 Length</label>
                    <input 
                        type="number" 
                        value={body.length} 
                        onChange={(e) => handleChange('length', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                    />
                </div>
            )}

            {!body.isParticle && (body.type === BodyType.CIRCLE || body.type === BodyType.ARC) && (
                <div>
                    <label className="block text-[10px] text-slate-400 mb-1 uppercase">半径 Radius</label>
                    <input 
                        type="number" 
                        value={body.radius} 
                        onChange={(e) => handleChange('radius', e.target.value)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
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
                            className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                        />
                    </div>
                    <div>
                        <label className="block text-[10px] text-slate-400 mb-1 uppercase">高度 Height</label>
                        <input 
                            type="number" 
                            value={body.height} 
                            onChange={(e) => handleChange('height', e.target.value)}
                            className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                        />
                    </div>
                </div>
            )}

            <div>
                 <label className="block text-[10px] text-slate-400 mb-1 uppercase">位置 (x, y)</label>
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
                 <label className="block text-[10px] text-slate-400 mb-1 uppercase">角度 (deg)</label>
                 <input 
                        type="number"
                        value={(body.angle * 180 / Math.PI).toFixed(1)} 
                        onChange={(e) => handleChange('angle', parseFloat(e.target.value) * Math.PI / 180)}
                        className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1 text-sm outline-none"
                    />
            </div>
        </div>
      </div>
    </div>
  );
};

export default PropertiesPanel;
