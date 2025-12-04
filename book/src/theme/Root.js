import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Default implementation, that you can customize
export default function Root({children}) {
  const {siteConfig} = useDocusaurusContext();
  const isHomePage = location.pathname === '/' || location.pathname === '/index.html';

  React.useEffect(() => {
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
