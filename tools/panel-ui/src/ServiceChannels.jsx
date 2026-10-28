const CHANNELS = [
  ['ir:3', 'Статус моторов'],
  ['ir:4', 'Пылесборник'],
  ['ir:6', 'Статус вентилятора'],
  ['ir:7', 'Ток основной щётки'],
  ['ir:8', 'Ток боковой щётки'],
  ['position3d:0', 'Гироскоп'],
  ['position3d:1', 'Одометрия + гироскоп'],
  ['position3d:4', 'Одометрия 3D'],
]
const MOTORS = ['Вентилятор', 'Основная щётка', 'Боковая щётка']
const array = (values) => `[${values.map((v) => Number.isFinite(v) ? v : '—').join(', ')}]`

function format(data) {
  if (!data) return null
  if (Array.isArray(data.voltages) && Array.isArray(data.ranges)) {
    return data.voltages.length || data.ranges.length
      ? `voltages ${array(data.voltages)} · ranges ${array(data.ranges)}` : null
  }
  if (Array.isArray(data.pos) && Array.isArray(data.vel)) {
    return `pos ${array(data.pos)} · vel ${array(data.vel)} · stall ${data.stall}`
  }
  return null
}

export default function ServiceChannels({ channels = {} }) {
  return (
    <details className="service-channels" role="group" aria-label="Служебные каналы">
      <summary>Служебные каналы</summary>
      <p className="sensor-note">Сырые поля Player: единицы и значения статусов не подтверждены. Последний кадр не гарантирует свежесть.</p>
      <table className="sensor-table" aria-label="Служебная телеметрия"><tbody>
        {CHANNELS.map(([key, name]) => {
          const text = format(channels?.[key])
          return <tr key={key}><td>{name} · {key}</td><td id={`service-${key}`} className={text === null ? 'missing' : ''}>{text ?? 'нет данных'}</td></tr>
        })}
        <tr><td>Кнопки · keypad:0</td><td id="service-keypad:0" className="missing">формат не подтверждён</td></tr>
        {MOTORS.map((name, i) => <tr key={i}><td>{name} · motor:{i}</td><td id={`service-motor:${i}`} className="missing">подписка отключена: возможны побочные эффекты</td></tr>)}
      </tbody></table>
    </details>
  )
}
