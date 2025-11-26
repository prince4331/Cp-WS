import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import Layout from './components/layout/Layout'
import HomePage from './pages/HomePage'
import ManualControl from './pages/ManualControl'
import AutoClean from './pages/AutoClean'
import MapPage from './pages/MapPage'
import Diagnostics from './pages/Diagnostics'
import Settings from './pages/Settings'

function App() {
  return (
    <Router future={{ v7_startTransition: true, v7_relativeSplatPath: true }}>
      <Layout>
        <Routes>
          <Route path="/" element={<HomePage />} />
          <Route path="/manual" element={<ManualControl />} />
          <Route path="/auto" element={<AutoClean />} />
          <Route path="/map" element={<MapPage />} />
          <Route path="/diagnostics" element={<Diagnostics />} />
          <Route path="/settings" element={<Settings />} />
        </Routes>
      </Layout>
    </Router>
  )
}

export default App
