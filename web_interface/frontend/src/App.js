import logo from './logo.svg';
import './App.css';
import React, { useEffect, useState } from 'react';
import ReactDOM from "react-dom/client";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import axios from 'axios'

function App() {
  const [getMessage, setGetMessage] = useState({})

  useEffect(()=>{
    axios.get('http://localhost:5000/flask/hello').then(response => {
      console.log("SUCCESS", response)
      setGetMessage(response)
    }).catch(error => {
      console.log(error)
    })

  }, [])
  return (
    <div className="App">
      <header className="App-header">
        <p>React + Flask Tutorial</p>
        <form method="post" action="localhost:5000/command_buttons">
          <button type="input" name="initiate_bite" value="initiate_bite"/>
          <button type="input" name="acquire_bite" value="acquire_bite"/>
          <button type="input" name="emergency_stop" value="emergency_stop"/>
          <button type="input" name="end_meal" value="end_meal"/>
        </form>
        <div>{getMessage.status === 200 ? 
          <h3>{getMessage.data.message}</h3>
          :
          <h3>LOADING</h3>}</div>
      </header>
    </div>
  );
}

export default App;