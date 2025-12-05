import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './onboarding.module.css';

export default function Logout() {
  return (
    <Layout title="Logged Out" description="You have been logged out">
      <div className={styles.onboardingContainer}>
        <div className={styles.onboardingCard}>
          <div className={styles.header}>
            <h1>See You Soon! 👋</h1>
            <p>
              You have been successfully logged out. 
              We hope to see you back soon to continue your learning journey.
            </p>
          </div>
          <div className={styles.navigationButtons}>
            <Link
              className={`${styles.button} ${styles.buttonPrimary}`}
              to="/auth">
              Login Again
            </Link>
            <Link
              className={`${styles.button} ${styles.buttonSecondary}`}
              to="/">
              Go Home
            </Link>
          </div>
        </div>
      </div>
    </Layout>
  );
}
