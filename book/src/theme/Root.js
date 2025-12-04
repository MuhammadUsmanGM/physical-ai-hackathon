import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Default implementation, that you can customize
export default function Root({children}) {
  const {siteConfig} = useDocusaurusContext();
  // Only access location in browser environment
  const isHomePage = ExecutionEnvironment.canUseDOM && 
    (window.location.pathname === '/' || 
     window.location.pathname === '/index.html' ||
     window.location.pathname === '/physical-ai-hackathon/' ||
     window.location.pathname === '/physical-ai-hackathon/index.html');

  React.useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) return;
    
    if (isHomePage) {
      document.documentElement.classList.add('home');
    } else {
      document.documentElement.classList.remove('home');
    }

    // Cleanup on unmount
    return () => {
      document.documentElement.classList.remove('home');
    };
  }, [isHomePage]);

  return (
    <AuthProvider>
      {isHomePage && <div className="mechanical-bg"></div>}
      {children}
    </AuthProvider>
  );
}
