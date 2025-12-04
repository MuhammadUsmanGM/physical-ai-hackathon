import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <div className="mechanical-bg"></div>
      {children}
    </AuthProvider>
  );
}
