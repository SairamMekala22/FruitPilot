import React, { useState } from 'react';
import { NavLink } from 'react-router-dom';
import { Home, Map, Bone as Drone, BarChart2, Settings, Menu, X, Citrus as Fruit } from 'lucide-react';

const Sidebar = () => {
  const [expanded, setExpanded] = useState(true);
  const [mobileOpen, setMobileOpen] = useState(false);
  
  const toggleSidebar = () => setExpanded(!expanded);
  const toggleMobileSidebar = () => setMobileOpen(!mobileOpen);

  const NavItem = ({ to, icon: Icon, label }: { to: string, icon: React.ElementType, label: string }) => (
    <NavLink 
      to={to} 
      className={({ isActive }) => `
        flex items-center px-4 py-3 text-gray-600 hover:bg-emerald-50 hover:text-emerald-700 rounded-lg transition-colors
        ${isActive ? 'bg-emerald-50 text-emerald-700 font-medium' : ''}
      `}
      onClick={() => setMobileOpen(false)}
    >
      <Icon size={20} className="flex-shrink-0" />
      <span className={`ml-3 ${expanded ? 'block' : 'hidden'}`}>{label}</span>
    </NavLink>
  );

  return (
    <>
      {/* Mobile overlay */}
      {mobileOpen && (
        <div 
          className="fixed inset-0 bg-black bg-opacity-50 z-20 md:hidden"
          onClick={toggleMobileSidebar}
        />
      )}
      
      {/* Mobile menu button */}
      <button 
        className="fixed top-4 left-4 p-2 rounded-md bg-white shadow-md text-gray-700 z-30 md:hidden" 
        onClick={toggleMobileSidebar}
      >
        <Menu size={24} />
      </button>
      
      {/* Sidebar */}
      <aside 
        className={`
          bg-white border-r border-gray-200 flex flex-col z-30
          ${mobileOpen ? 'fixed inset-y-0 left-0 w-64' : 'hidden'} 
          md:flex md:static md:w-auto md:transform-none
          transition-all duration-300 ease-in-out
        `}
      >
        {/* Sidebar header */}
        <div className="flex items-center justify-between p-4 border-b border-gray-200">
          <div className="flex items-center space-x-2">
            <div className="w-8 h-8 rounded-lg bg-emerald-600 text-white flex items-center justify-center">
              <Fruit size={20} />
            </div>
            <h2 className={`font-bold text-xl text-gray-800 ${expanded ? 'block' : 'hidden'}`}>Fruit Pilot</h2>
          </div>
          
          <button 
            onClick={toggleMobileSidebar} 
            className="p-1 rounded-md hover:bg-gray-100 md:hidden"
          >
            <X size={20} className="text-gray-600" />
          </button>
          
          <button 
            onClick={toggleSidebar} 
            className="p-1 rounded-md hover:bg-gray-100 hidden md:block"
          >
            <Menu size={20} className="text-gray-600" />
          </button>
        </div>
        
        {/* Nav items */}
        <nav className="flex-1 overflow-y-auto p-3 space-y-1">
          <NavItem to="/" icon={Home} label="Dashboard" />
          <NavItem to="/mission-planner" icon={Map} label="Mission Planner" />
          <NavItem to="/drone-details/current" icon={Drone} label="Drone Control" />
          {/* <NavItem to="/analytics" icon={BarChart2} label="Analytics" />
          <NavItem to="/settings" icon={Settings} label="Settings" /> */}
        </nav>
      </aside>
    </>
  );
};

export default Sidebar;