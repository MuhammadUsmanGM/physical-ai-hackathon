import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import OnboardingGuard from '@site/src/components/OnboardingGuard';
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

  // Check if current page is auth or onboarding (don't protect these)
  const isAuthPage = ExecutionEnvironment.canUseDOM && 
    (window.location.pathname.includes('/auth') || 
     window.location.pathname.includes('/onboarding') ||
     window.location.pathname.includes('/logout'));

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
      {isAuthPage ? (
        children
      ) : (
        <OnboardingGuard>
          {children}
        </OnboardingGuard>
      )}
    </AuthProvider>
  );
}
