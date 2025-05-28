import { useEffect, useState, useRef } from 'react';
import io from 'socket.io-client';
import {
  LineChart, Line, XAxis, YAxis,
  CartesianGrid, Tooltip, ResponsiveContainer,
} from 'recharts';

const socket = io('http://localhost:8000');

// find average speed and graph it in real-time

function averageSpeed(samples) {
  if (samples.length === 0) return 0;
  const total = samples.reduce((sum, { speed }) => sum + speed, 0);
  return total / samples.length;
}

function App() {
  const [samples, setSamples] = useState([]);
  const counter = useRef(0); // x-axis index (avoid re-render spam)

  useEffect(() => {
    /** Push incoming speed into rolling buffer (max 200 pts). */
    socket.on('update', ({ speed }) => {
      counter.current += 1;
      console.log(`Received speed: ${speed} at t=${counter.current}`);
      setSamples((buf) => {
        const next = [...buf, { t: counter.current, speed }];
        return next.length > 200 ? next.slice(-200) : next;
      });
    });

    return () => socket.off('update');
  }, []);

  return (
    <div className="p-10">
      <h1 className="text-3xl font-bold mb-6">Real-Time Speed Chart</h1>

      <ResponsiveContainer width="100%" height={300}>
        <LineChart data={samples}>
          <CartesianGrid strokeDasharray="3 3" />
          <XAxis dataKey="t" tick={false} />
          <YAxis domain={[0, 3]} />
          <Tooltip />
          <Line
            type="monotone"
            dataKey="speed"
            strokeWidth={2}
            dot={true}
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}

export default App;
