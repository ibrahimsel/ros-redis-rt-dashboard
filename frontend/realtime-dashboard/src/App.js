import { useEffect, useState } from 'react';
import io from 'socket.io-client';

const socket = io('http://localhost:8000');

function App() {
  const [data, setData] = useState({ speed: 0 });

  useEffect(() => {
    socket.on('update', (data) => {
      console.log('🔥 Got update:', data);
      setData(data);
    });
    return () => socket.off('update');
  }, []);

  return (
    <div className="p-10">
      <h1 className="text-3xl font-bold">🚗 Real-Time Dashboard</h1>
      <p className="mt-4">Speed: {data.speed}</p>
    </div>
  );
}

export default App;
