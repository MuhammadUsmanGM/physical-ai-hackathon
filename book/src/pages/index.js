import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero heroBanner', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle delay-1">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg delay-2"
            to="/docs/module-01">
            Begin Your Physical AI Journey
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
        <section className="featuresSection">
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="card floating delay-1">
                  <div className="feature-icon">🤖</div>
                  <h3>Physical AI</h3>
                  <p>Dive deep into the revolutionary field of embodied intelligence, where digital minds meet the physical world.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="card floating delay-2">
                  <div className="feature-icon">🔧</div>
                  <h3>Master Robotics</h3>
                  <p>Experience hands-on learning with industry-standard platforms like ROS 2, Gazebo, Unity, and NVIDIA Isaac.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="card floating">
                  <div className="feature-icon">🚀</div>
                  <h3>Intelligent Systems</h3>
                  <p>Design and implement autonomous humanoid robots that interact naturally with humans and environments.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
        
        <section className="featuresSection" style={{background: 'var(--ifm-background-surface-color)'}}>
          <div className="container padding-vert--xl text--center">
            <h2 style={{fontSize: '2.5rem', marginBottom: '3rem'}}>🎓 Complete Course Modules</h2>
            
            <div className="module-grid">
              <Link to="/docs/module-01" className="card module-card">
                <div className="feature-icon" style={{fontSize: '2rem', width: '60px', height: '60px'}}>🔍</div>
                <h3>Introduction to Physical AI</h3>
                <p>Foundations of embodiment, sensors, and physical reasoning.</p>
              </Link>

              <Link to="/docs/module-02" className="card module-card">
                <div className="feature-icon" style={{fontSize: '2rem', width: '60px', height: '60px'}}>📡</div>
                <h3>ROS 2 Fundamentals</h3>
                <p>Master the robotic nervous system with nodes, topics, and services.</p>
              </Link>

              <Link to="/docs/module-03" className="card module-card">
                <div className="feature-icon" style={{fontSize: '2rem', width: '60px', height: '60px'}}>🎬</div>
                <h3>Simulation (Gazebo & Unity)</h3>
                <p>Create realistic digital twins and master sim-to-real transfer.</p>
              </Link>

              <Link to="/docs/module-04" className="card module-card">
                <div className="feature-icon" style={{fontSize: '2rem', width: '60px', height: '60px'}}>🧠</div>
                <h3>NVIDIA Isaac Platform</h3>
                <p>Leverage Isaac Sim, Isaac ROS, and Jetson for edge AI.</p>
              </Link>

              <Link to="/docs/module-05" className="card module-card">
                <div className="feature-icon" style={{fontSize: '2rem', width: '60px', height: '60px'}}>💬</div>
                <h3>Vision-Language-Action</h3>
                <p>Build end-to-end VLA systems with LLMs and multi-modal AI.</p>
              </Link>
              
              <Link to="/docs/module-05" className="card module-card" style={{borderColor: 'var(--color-accent-purple)'}}>
                <div className="feature-icon" style={{fontSize: '2rem', width: '60px', height: '60px', background: 'rgba(213, 0, 249, 0.1)', color: 'var(--color-accent-purple)'}}>🏆</div>
                <h3 style={{color: 'var(--color-accent-purple)'}}>Capstone Project</h3>
                <p>Build a complete autonomous humanoid assistant.</p>
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}