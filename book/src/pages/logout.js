import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './onboarding.module.css'; // Reusing onboarding styles for consistency

export default function Logout() {
  return (
    <Layout title="Logged Out" description="You have been logged out">
      <div className={styles.container}>
        <div className={styles.card}>
          <h1>See You Soon! 👋</h1>
          <p>
            You have been successfully logged out. 
            We hope to see you back soon to continue your learning journey.
          </p>
          <div className={styles.actions}>
            <Link
              className="button button--primary button--lg"
              to="/auth">
              Login Again
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/">
              Go Home
            </Link>
          </div>
        </div>
      </div>
    </Layout>
  );
}
