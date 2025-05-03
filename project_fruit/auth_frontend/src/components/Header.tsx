import React from 'react';
import { Bell, User, Settings } from 'lucide-react';

const Header = () => {
  return (
    <header className="bg-white border-b border-gray-200 flex items-center justify-between px-4 py-3 shadow-sm z-10">
      <div>
        <h1 className="text-xl font-semibold text-gray-800">Fruit Pilot</h1>
      </div>
      
      <div className="flex items-center space-x-4">
        <button className="p-2 rounded-full hover:bg-gray-100 transition-colors">
          <Bell size={20} className="text-gray-600" />
        </button>
        <button className="p-2 rounded-full hover:bg-gray-100 transition-colors">
          <Settings size={20} className="text-gray-600" />
        </button>
        <div className="flex items-center space-x-2">
          <div className="w-8 h-8 rounded-full bg-emerald-600 flex items-center justify-center text-white">
            <User size={16} />
          </div>
          <span className="hidden md:inline text-sm font-medium">John Doe</span>
        </div>
      </div>
    </header>
  );
};

export default Header;