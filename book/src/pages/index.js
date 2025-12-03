import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/01-Introduction-to-Physical-AI/">
            Start Your Journey Now 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h2>Learn Physical AI</h2>
                  <p>Explore the fundamentals of AI systems in the physical world and embodied intelligence.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h2>Master Robotics</h2>
                  <p>Gain hands-on experience with ROS 2, Gazebo, Unity, and NVIDIA Isaac platforms.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h2>Build Intelligent Systems</h2>
                  <p>Create autonomous humanoid systems capable of natural human interaction.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
        
        <section className={styles.courseOverview}>
          <div className="container padding-vert--xl text--center">
            <h2>Course Modules</h2>
            <p>Follow the structured course modules to master Physical AI and Humanoid Robotics:</p>
            <div className="row">
              <div className="col">
                <Link to="/docs/01-Introduction-to-Physical-AI/" className="button button--primary button--outline">
                  Introduction to Physical AI
                </Link>
              </div>
              <div className="col">
                <Link to="/docs/02-The-Robotic-Nervous-System/" className="button button--primary button--outline">
                  ROS 2 Fundamentals
                </Link>
              </div>
              <div className="col">
                <Link to="/docs/03-The-Digital-Twin/" className="button button--primary button--outline">
                  Simulation (Gazebo & Unity)
                </Link>
              </div>
            </div>
            <div className="row margin-top--lg">
              <div className="col">
                <Link to="/docs/04-The-AI-Robot-Brain/" className="button button--primary button--outline">
                  NVIDIA Isaac Platform
                </Link>
              </div>
              <div className="col">
                <Link to="/docs/05-Vision-Language-Action/" className="button button--primary button--outline">
                  Vision-Language-Action
                </Link>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}