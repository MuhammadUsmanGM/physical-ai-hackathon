import { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../contexts/AuthContext';

/**
 * OnboardingGuard - Protects routes by ensuring users complete onboarding
 * 
 * Usage: Wrap components that require onboarding completion
 * 
 * @param {Object} props
 * @param {React.ReactNode} props.children - Child components to render if onboarding is complete
 * @returns {React.ReactNode} Children if onboarding complete, null otherwise (redirects to onboarding)
 */
export default function OnboardingGuard({ children }) {
  const { user, loading } = useAuth();
  const history = useHistory();
  const onboardingUrl = useBaseUrl('/onboarding');

  useEffect(() => {
    // Don't redirect while loading user data
    if (loading) return;

    // If user is logged in but hasn't completed onboarding, redirect
    if (user && !user.onboarding_completed) {
      history.push(onboardingUrl);
    }
  }, [user, loading, history, onboardingUrl]);

  // Show nothing while loading or redirecting
  if (loading || (user && !user.onboarding_completed)) {
    return null;
  }

  // Render children if onboarding is complete or user is not logged in
  return children;
}
