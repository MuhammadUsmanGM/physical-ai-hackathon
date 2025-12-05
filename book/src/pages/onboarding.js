import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './onboarding.module.css';

export default function Onboarding() {
  return (
    <Layout title="Welcome" description="Welcome to Physical AI & Humanoid Robotics">
      <div className={styles.container}>
        <div className={styles.card}>
          <h1>Welcome to the Course! 🚀</h1>
          <p>
            You have successfully created your account. We are excited to have you on board 
            for this journey into Physical AI and Humanoid Robotics.
          </p>
          <div className={styles.actions}>
            <Link
              className="button button--primary button--lg"
              to="/docs/module-01">
              Start Learning
            </Link>
          </div>
        </div>
      </div>
    </Layout>
  );
}
