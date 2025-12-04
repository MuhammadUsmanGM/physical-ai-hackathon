import React from 'react';
import NavbarNavLink from '@theme-original/NavbarItem/NavbarNavLink';
import { useAuth } from '@site/src/contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

export default function NavbarNavLinkWrapper(props) {
  const { user, logout } = useAuth();
  const history = useHistory();

  // Check if this is the auth link
  if (props.className?.includes('auth-nav-link')) {
    if (user) {
      // If logged in, show "Logout" instead of "Get Started"
      // Or show "Profile"
      return (
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <span style={{ fontSize: '14px', fontWeight: '600', color: 'var(--ifm-color-primary)' }}>
            Hi, {user.name.split(' ')[0]}
          </span>
          <button
            onClick={() => {
              logout();
              history.push('/');
            }}
            className="clean-btn"
            style={{
              padding: '6px 12px',
              borderRadius: '8px',
              backgroundColor: 'rgba(239, 68, 68, 0.1)',
              color: 'rgb(239, 68, 68)',
              fontSize: '13px',
              fontWeight: '600',
              border: '1px solid rgba(239, 68, 68, 0.2)',
              cursor: 'pointer',
              transition: 'all 0.2s ease'
            }}
          >
            Logout
          </button>
        </div>
      );
    }

    // If not logged in, show styled "Get Started" button
    return (
      <NavbarNavLink
        {...props}
        style={{
          backgroundColor: 'var(--ifm-color-primary)',
          color: 'white',
          padding: '8px 16px',
          borderRadius: '8px',
          fontWeight: '600',
          transition: 'all 0.2s ease',
          ...props.style,
        }}
        className={`${props.className} get-started-btn`}
      />
    );
  }

  return <NavbarNavLink {...props} />;
}
